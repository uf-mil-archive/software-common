from __future__ import division

import math
import time

import pygame
import pygame.gfxdraw

from imagenex_852 import driver


d = driver.Device('/dev/ttyUSB0')
disp = pygame.display.set_mode((512, 512))
while True:
    time.sleep(.03)
    d.send_switch_data_command(train_angle_degrees=0, sector_width_degrees=360, step_size_degrees=3)
    res = d.read_sonar_return_data()
    print int(res['head_position_degrees'])
    for i, x in reversed(list(enumerate(res['echo_data']))):
        r = int(res['head_position_degrees'])
        #print (256-i, 256-i, 2*i, 2*i), x
        #x *= 10
        #if x >= 255: x = 255
        #if i < 3: continue
        #i *= 2
        #pygame.draw.arc(disp, (x, x, x), (256-i, 256-i, 2*i, 2*i), r, r+.6, 0)
        pygame.gfxdraw.pie(disp, 256, 256, i, r, r+10, (x, x, x, 100))
    pygame.draw.line(disp, (0, 255, 0), (256, 256-25*3//2), (256, 256+25*3//2))
    pygame.display.update()
