#!/usr/bin/python

from __future__ import division

from twisted.internet import defer, reactor, serialport, protocol
from twisted.protocols import basic

import txros
from txros import util, variable

from thruster_handling.msg import ThrusterCommand


class LoadCellProtocol(basic.LineOnlyReceiver):
    def __init__(self):
        self.measurement_event = variable.Event()
    def lineReceived(self, line):
        x = int(line)
        V3 = x * (3/2**12) # convert 12 bits to 0-3 volts (output of voltage divider)
        V2 = 3 * V3 # convert output of voltage divider to output of bias amp
        V1 = 2 * V2 - 12.8 # convert output of bias amp to output of instrument amp
        Vload = V1/804 # convert output of instrument amp to output of loadcell
        force = (Vload + 1.27e-3) * 100/(29.955e-3)
        force = force * (0.45359237 * 9.80665) # lbf -> N
        
        #print '%+.03f' % force
        
        self.measurement_event.happened(force)

class LoadCell(object):
    def __init__(self, nh, port):
        self._protocol = LoadCellProtocol()
        self.measurement_event = self._protocol.measurement_event
        self._port = serialport.SerialPort(self._protocol, port, reactor, 9600)

@util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv('thruster_calibrator')
    
    pub = nh.advertise('/thrusters/command/test', ThrusterCommand)
    
    port = yield nh.get_param('~port')
    
    lc = LoadCell(nh, port)
    
    power_holder = [0]
    
    @util.cancellableInlineCallbacks
    def set_thruster_thread():
        while True:
            pub.publish(ThrusterCommand(force=power_holder[0]))
            yield util.sleep(.1)
    set_thruster_thread()
    
    for i in xrange(1, -20, -1):
        power = i/20
        
        power_holder[0] = power
        
        yield util.sleep(1)
    
    res = []
    
    for i in xrange(-20, 20+1):
        power = i/20
        
        power_holder[0] = power
        yield util.sleep(2)
        
        s = 0
        for j in xrange(400):
            s += (yield lc.measurement_event.get_deferred())[0]
        s /= 400
        
        if i == 0: zero_s = s
        
        print power, s
        
        res.append((s, power))
    
    print [(-(x-zero_s), y) for x, y in res]
    
    power_holder[0] = 0
    
    yield util.sleep(1)
    
util.launch_main(main)
