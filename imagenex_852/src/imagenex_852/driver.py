import serial

class Device(object):
    def __init__(self, port_filename):
        self.port = serial.Serial(port_filename, 115200, 8, 'N', 1)
    
    switch_data_command_desc = [
        # name, default, allowed_values
        ('range_meters', 5, [5, 10, 20, 30, 40, 50]),
        ('reverse', False, [False, True]),
        ('start_gain_dB', 32, range(0, 40+1)),
        ('train_angle_degrees', 0, [x for x in xrange(-210, 210+1) if x % 3 == 0]),
        ('sector_width_degrees', 30, [x for x in xrange(0, 360+1) if x % 3 == 0]),
        ('step_size_degrees', 3, [0, 3, 6]),
        ('pulse_length_usec', 1, range(1, 255+1)),
        ('data_points', 500, [250, 500]),
        ('switch_delay_msec', 0, [x for x in xrange(0, 255+1) if x != 253]),
        ('frequency_kHz', 675, [675, 850]),
    ]
    def send_switch_data_command(self, **kwargs):
        c = {}
        for name, default, allowed_values in self.switch_data_command_desc:
            # make sure options are valid
            c[name] = kwargs.get(name, default)
            if c[name] not in allowed_values:
                raise ValueError('invalid value: %s = %r' % (name, c[name]))
        
        inner_data = ''
        inner_data += chr(0x10)                     # Byte 2; Head ID; 0x10 only
        inner_data += chr(c['range_meters'])        # Byte 3; Range; 5 to 50 Meters; 5,10,20,30,40,50 Meters supported
        inner_data += chr(0)                        # Byte 4; Reserved
        inner_data += chr(2**6 if c['reverse'] else 0) # Byte 5; Rev
        inner_data += chr(0)                        # Byte 6; Reserved
        inner_data += chr(0)                        # Byte 7; Reserved
        inner_data += chr(c['start_gain_dB'])       # Byte 8; Start Gain; 0 to 40dB in 1dB increments
        inner_data += chr(0)                        # Byte 9; Reserved
        inner_data += chr(20)                       # Byte 10; Absorption; 20 = 0.2 dB/m 675,850 kHz
        inner_data += chr((c['train_angle_degrees'] + 210)//3) # Byte 11; Train Angle
        inner_data += chr(c['sector_width_degrees']//3) # Byte 12; Sector Width
        inner_data += chr(c['step_size_degrees']//3) # Byte 13; Step Size
        inner_data += chr(c['pulse_length_usec'])   # Byte 14; Pulse Length
        inner_data += chr(0)                        # Byte 15; Reserved
        inner_data += chr(0)                        # Byte 16; Reserved
        inner_data += chr(0)                        # Byte 17; Reserved
        inner_data += chr(0)                        # Byte 18; Reserved
        inner_data += chr(c['data_points']//10)     # Byte 19; Data Points; 25 - 250 data points are returned by the head; 50 - 500 data points are returned by the head
        inner_data += chr(0)                        # Byte 20; Reserved
        inner_data += chr(0)                        # Byte 21; Reserved
        inner_data += chr(0)                        # Byte 22; Reserved
        inner_data += chr(0)                        # Byte 23; Reserved
        inner_data += chr(c['switch_delay_msec'])   # Byte 24; Switch Delay
        inner_data += chr(1 if c['frequency_kHz'] == 850 else 0) # Byte 25; Frequency; 0 = 675 kHz; 1 = 850 kHz
        
        assert '\xfd' not in inner_data, 'inner_data cannot contain 0xFD byte'
        wrapped_data = '\xfe\x44' + inner_data + '\xfd'
        self.port.write(wrapped_data)
    
    def read_sonar_return_data(self):
        res = {}
        
        header = self.port.read(3) # Byte 0 - 2
        while header not in ['IMX', 'IGX']:
            header = header[1:] + self.port.read(1)
        
        head_id = ord(self.port.read(1)) # Byte 3
        if head_id != 0x10: print 'head_id =', head_id
        
        serial_status = ord(self.port.read(1)) # Byte 4
        res['sonar_head'] = bool(serial_status & 0b1)
        res['switches_accepted'] = bool(serial_status & 0b1000000)
        res['character_overrun'] = bool(serial_status & 0b10000000)
        
        head_position_raw = map(ord, self.port.read(2)) # Byte 5 - 6
        head_position_high = (head_position_raw[1] & 0x3e) >> 1
        head_position_low = ((head_position_raw[1] & 0x01) << 7) | (head_position_raw[0] & 0x7f)
        head_position = (head_position_high << 8) + head_position_low
        res['head_position_degrees'] = .15 * (head_position - 1400)
        
        res['range_meters'] = ord(self.port.read(1)) # Byte 7
        
        reserved1 = ord(self.port.read(1)) # Byte 8
        #if reserved1 != 0: print 'reserved1 =', reserved1 # usually 30 instead of 0
        
        reserved2 = ord(self.port.read(1)) # Byte 9
        if reserved2 != 0: print 'reserved2 =', reserved2
        
        data_bytes_raw = map(ord, self.port.read(2)) # Byte 10 - 11
        data_bytes_high = (data_bytes_raw[1] & 0x7e) >> 1
        data_bytes_low = (data_bytes_raw[1] & 0x7e) >> 1
        res['data_bytes'] = (data_bytes_high << 8) + data_bytes_low
        
        res['echo_data'] = map(ord, self.port.read(252 if header == 'IMX' else 500)) # Byte 12 - (N-2)
        
        termination_byte = ord(self.port.read(1)) # Byte (N-1)
        if termination_byte != 0xFC: print 'termination_byte =', termination_byte
        
        return res
