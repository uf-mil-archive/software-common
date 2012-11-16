import serial

class Device(object):
    def __init__(self, port_filename):
        self.port = serial.Serial(port_filename, 115200, 8, 'N', 1)
    
    def send_switch_data_command(self,
        range_meters=5, reverse=False, start_gain_dB=32, train_angle_degrees=0,
        sector_width_degrees=30, step_size_degrees=3, pulse_length_usec=1,
        data_points=500, switch_delay_msec=0, frequency_kHz=675,
    ):
        inner_data = ''
        inner_data += chr(0x10)                     # Byte 2; Head ID; 0x10 only
        assert range_meters in [5, 10, 20, 30, 40, 50]
        inner_data += chr(range_meters)             # Byte 3; Range; 5 to 50 Meters; 5,10,20,30,40,50 Meters supported
        inner_data += chr(0)                        # Byte 4; Reserved
        inner_data += chr(2**6 if reverse else 0)   # Byte 5; Rev
        inner_data += chr(0)                        # Byte 6; Reserved
        inner_data += chr(0)                        # Byte 7; Reserved
        assert 0 <= start_gain_dB <= 40
        inner_data += chr(start_gain_dB)            # Byte 8; Start Gain; 0 to 40dB in 1dB increments
        inner_data += chr(0)                        # Byte 9; Reserved
        inner_data += chr(20)                       # Byte 10; Absorption; 20 = 0.2 dB/m 675,850 kHz
        assert -210 <= train_angle_degrees <= 210 and train_angle_degrees % 3 == 0
        inner_data += chr((train_angle_degrees + 210)//3) # Byte 11; Train Angle
        assert 0 <= sector_width_degrees <= 360 and sector_width_degrees % 3 == 0
        inner_data += chr(sector_width_degrees//3)  # Byte 12; Sector Width
        assert step_size_degrees in [0, 3, 6]
        inner_data += chr(step_size_degrees//3)     # Byte 13; Step Size
        assert 1 <= pulse_length_usec <= 255
        inner_data += chr(pulse_length_usec)        # Byte 14; Pulse Length
        inner_data += chr(0)                        # Byte 15; Reserved
        inner_data += chr(0)                        # Byte 16; Reserved
        inner_data += chr(0)                        # Byte 17; Reserved
        inner_data += chr(0)                        # Byte 18; Reserved
        assert data_points in [250, 500]
        inner_data += chr(data_points//10)          # Byte 19; Data Points; 25 - 250 data points are returned by the head; 50 - 500 data points are returned by the head
        inner_data += chr(0)                        # Byte 20; Reserved
        inner_data += chr(0)                        # Byte 21; Reserved
        inner_data += chr(0)                        # Byte 22; Reserved
        inner_data += chr(0)                        # Byte 23; Reserved
        inner_data += chr(0)                        # Byte 24; Switch Delay
        assert frequency_kHz in [675, 850]
        inner_data += chr(1 if frequency_kHz == 850 else 0) # Byte 25; Frequency; 0 = 675 kHz; 1 = 850 kHz
        
        if '\xfd' in inner_data:
            raise ValueError('inner_data cannot contain 0xFD byte')
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
