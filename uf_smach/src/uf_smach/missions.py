import smach
import smach_ros
from uf_smach import common_states

_mission_factories = dict()

def register_factory(name, factory):
    _mission_factories[name] = factory

def make_mission(shared, name, config):
    return _mission_factories[name](shared, config)

_mission_configs = dict()

def register_config(name, config):
    _mission_configs[name] = config

def make_mission_state_machine(shared, name):
    config = _mission_configs[name]
    sm = smach.StateMachine(outcomes=['succeeded', 'contigency_plan', 'preempted'],
                            output_keys=['contigency_plan'])
    with sm:
        sm_con = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                   default_outcome='failed',
                                   outcome_map={'succeeded': { 'MISSION': 'succeeded'}},
                                   child_termination_cb=lambda so: True)
        with sm_con:
            smach.Concurrence.add('MISSION', make_mission(shared, config['factory'], config))
            smach.Concurrence.add('TIMEOUT', common_states.SleepState(config['timeout']))

        smach.StateMachine.add('MISSION_AND_TIMEOUT', 
                               sm_con,
                               transitions={'succeeded': 'succeeded',
                                                'failed': 'CONTIGENCY_PLAN'})
        smach.StateMachine.add('CONTIGENCY_PLAN',
                               common_states.SetUserDataState(contigency_plan=config.get('contigency_plan')),
                               transitions={'succeeded': 'contigency_plan'})
    return sm

def make_plan_state_machine(shared, names):
    sm = smach.Sequence(['succeeded', 'contigency_plan', 'preempted'], 'succeeded')
    with sm:
        for name in names:
            smach.Sequence.add(name.upper(),
                               make_mission_state_machine(shared, name))
    return sm

def run_plans(plans):
    sis_list = [smach_ros.IntrospectionServer(name, sm, '/'+name.upper())
                for name, sm in plans.iteritems()]
    apply(smach_ros.IntrospectionServer.start, sis_list)

    current_plan = 'main'
    while current_plan is not None:
        sm = plans[current_plan]
        sm.userdata.contigency_plan = None
        outcome = sm.execute()
        if outcome == 'succeeded':
            break
        elif outcome == 'contigency_plan':
            current_plan = sm.userdata.contigency_plan
        else:
            assert False, 'Unknown plan outcome'
            
    apply(smach_ros.IntrospectionServer.stop, sis_list)
    return outcome == 'succeeded'
