import smach
import smach_ros
import rospy
import actionlib
from uf_smach import common_states, legacy_vision_states
from uf_smach.msg import Plan, PlansStamped, RunMissionsAction, RunMissionsResult
from uf_smach.srv import ModifyPlan, ModifyPlanRequest
import uf_smach
from kill_handling.listener import KillListener
from std_msgs.msg import Header
from collections import namedtuple

_mission_factories = dict()

def register_factory(name, factory):
    _mission_factories[name] = factory

def get_missions():
    return _mission_factories.keys()

PlanEntry = namedtuple('PlanEntry', 'mission timeout contigency_plan path')

class PlanSet(object):
    def __init__(self, names):
        self._plans = dict((name, []) for name in names)

    def get_plan(self, plan):
        return self._plans[plan]

    def get_plans(self):
        return self._plans.keys()
    
    def make_sm(self, shared, allow_nonexistent_contigencies=False):
        sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
        with sm:
            for plan in _iterate_main_first(self._plans):
                plan_sm, contigency_outcomes = self._make_plan_sm(shared, plan)
                transitions = self._make_plan_transitions(plan, contigency_outcomes,
                                                          allow_nonexistent_contigencies)
                smach.StateMachine.add('PLAN_' + plan.upper(), plan_sm, transitions=transitions)
        return sm

    def _make_plan_transitions(self, plan, contigency_outcomes, allow_nonexistent_contigencies):
        transitions = {'succeeded': 'succeeded', 'preempted': 'preempted'}
        for outcome in contigency_outcomes:
            if outcome in self._plans:
                transitions[outcome] = 'PLAN_' + outcome.upper()
            elif outcome == 'no_contigency' or allow_nonexistent_contigencies:
                transitions[outcome] = 'failed'
            else:
                raise RuntimeError('Plan %s requires an unknown contigency plan %s' %
                                   (plan, outcome))
        return transitions

    def _make_plan_sm(self, shared, plan):
        entries = self._plans[plan]
        if len(entries) > 0:
            entry_sms, contigency_outcomes = zip(*[
                    self._make_mission_sm(shared, entry) for entry in entries])
        else:
            entry_sms = []
            contigency_outcomes = []
        contigency_outcomes_set = set(contigency_outcomes)
        sm = smach.Sequence(['succeeded'] + list(contigency_outcomes_set) + ['preempted'], 'succeeded')
        with sm:
            for entry, entry_sm, contigency_outcome in zip(entries, entry_sms, contigency_outcomes):
                if entry.path is not None:
                    smach.Sequence.add('PIPE_' + entry.mission.upper(),
                                       self._make_path_sm(shared, entry.path),
                                       transitions={'failed': contigency_outcome})
                smach.Sequence.add(entry.mission.upper(), entry_sm)
        return sm, contigency_outcomes

    def _make_mission_sm(self, shared, entry):
        mission_factory = _mission_factories[entry.mission]
        contigency_outcome = entry.contigency_plan
        if contigency_outcome is None:
            contigency_outcome = 'no_contigency'
        sm = smach.Concurrence(outcomes=['succeeded', 'preempted', contigency_outcome],
                               default_outcome=contigency_outcome,
                               outcome_map={'succeeded': { 'MISSION': 'succeeded' },
                                            'preempted': { 'MISSION': 'preempted',
                                                           'TIMEOUT': 'preempted'}},
                               child_termination_cb=lambda so: True)
        with sm:
            smach.Concurrence.add('MISSION', mission_factory(shared))
            smach.Concurrence.add('TIMEOUT', common_states.SleepState(entry.timeout))
        return sm, contigency_outcome

    def _make_path_sm(self, shared, path):
        if path in ('left' or 'right'):
            selector = legacy_vision_states.select_by_angle(path)
        else:
            selector = legacy_vision_states.select_first

        sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
        with sm:
            smach.Sequence.add('CENTER_PIPE',
                               legacy_vision_states.CenterObjectState(shared,
                                                                      'find2_down_camera',
                                                                      selector))
            smach.Sequence.add('ALIGN_PIPE',
                               legacy_vision_states.AlignObjectState(shared,
                                                                     'find2_down_camera',
                                                                     selector))
        return sm

def _iterate_main_first(items):
    if 'main' in items:
        yield 'main'
    for item in items:
        if item != 'main':
            yield item
    
class MissionServer(object):
    def __init__(self, plan_names):
        self._plans = PlanSet(plan_names)
        self._sm = None
        self._pub = rospy.Publisher('mission/plans', PlansStamped)
        self._srv = rospy.Service('mission/modify_plan', ModifyPlan, self._modify_plan)
        self._run_srv = actionlib.SimpleActionServer('mission/run', RunMissionsAction,
                                                     self.execute, False)
        self._run_srv.register_preempt_callback(self._on_preempt)
        self._run_srv.start()
        self._tim = rospy.Timer(rospy.Duration(.1), lambda _: self._publish_plans())
        self._kill_listener = KillListener(self._on_preempt)
        self._shared = uf_smach.util.StateSharedHandles()

    def get_plan(self, plan):
        return self._plans.get_plan(plan)

    def execute(self, goal):
        self._sm = self._plans.make_sm(self._shared)
        sis = smach_ros.IntrospectionServer('mission_planner', self._sm, '/SM_ROOT')
        sis.start()
        outcome = self._sm.execute()
        sis.stop()
        self._shared['moveto'].cancel_goal()
        if outcome == 'succeeded':
            self._run_srv.set_succeeded(RunMissionsResult(outcome))
        else:
            self._run_srv.set_preempted()

    def _on_preempt(self):
        if self._sm is not None:
            self._sm.request_preempt()
    
    def _publish_plans(self):
        self._pub.publish(PlansStamped(
                header=Header(stamp=rospy.Time.now()),
                plans=[Plan(name=name,
                            entries=[uf_smach.msg.PlanEntry(entry.mission, rospy.Duration(entry.timeout),
                                                            entry.contigency_plan, entry.path)
                                     for entry in self._plans.get_plan(name)])
                       for name in self._plans.get_plans()],
                available_missions=get_missions()))

    def _modify_plan(self, req):
        if req.plan not in self._plans.get_plans():
            return None
        plan = self._plans.get_plan(req.plan)
        entry = PlanEntry(req.entry.mission,
                          req.entry.timeout.to_sec(),
                          req.entry.contigency_plan if len(req.entry.contigency_plan) > 0 else None,
                          req.entry.path if req.entry.path != 'none' else None)
        if req.operation == ModifyPlanRequest.INSERT:
            if req.pos > len(plan):
                return None
            plan.insert(req.pos, entry)
        elif req.operation == ModifyPlanRequest.REMOVE:
            if req.pos >= len(plan):
                return None
            del plan[req.pos]
        else:
            return None
        return ()
