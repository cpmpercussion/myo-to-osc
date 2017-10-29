#
# Original work Copyright (c) 2014 Danny Zhu
# Modified work Copyright (c) 2017 Alvaro Villoslada, Fernando Cosentino
# 
# Licensed under the MIT license. See the LICENSE file for details.
#

import sys
import time
from myo_raw import MyoRaw
try:
    import pygame
    from pygame.locals import *
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False

if HAVE_PYGAME:
    w, h = 800, 600
    scr = pygame.display.set_mode((w, h))
    last_vals = None
    def plot(scr, vals, DRAW_LINES=True):
        global last_vals
        if last_vals is None:
            last_vals = vals
            return
        D = 5
        scr.scroll(-D)
        scr.fill((0, 0, 0), (w - D, 0, w, h))
        for i, (u, v) in enumerate(zip(last_vals, vals)):
            if DRAW_LINES:
                pygame.draw.line(scr, (0, 255, 0),
                                 (w - D, int(h/9 * (i+1 - u))),
                                 (w, int(h/9 * (i+1 - v))))
                pygame.draw.line(scr, (255, 255, 255),
                                 (w - D, int(h/9 * (i+1))),
                                 (w, int(h/9 * (i+1))))
            else:
                c = int(255 * max(0, min(1, v)))
                scr.fill((c, c, c), (w - D, i * h / 8, D, (i + 1) * h / 8 - i * h / 8))
        pygame.display.flip()
        last_vals = vals

def proc_emg(emg, moving, times=[]):
    if HAVE_PYGAME:
        # update pygame display
        plot(scr, [e / 500. for e in emg])
    else:
        print(emg)
    # print framerate of received data
    times.append(time.time())
    if len(times) > 20:
        # print((len(times) - 1) / (times[-1] - times[0]))
        times.pop(0)

def proc_battery(battery_level):
    print("Battery level: %d" % battery_level)
    if battery_level < 5:
        m.set_leds([255, 0, 0], [255, 0, 0])
    else:
        m.set_leds([128, 128, 255], [128, 128, 255])

m = MyoRaw(sys.argv[1] if len(sys.argv) >= 2 else None)
m.add_emg_handler(proc_emg)
m.add_battery_handler(proc_battery)
m.connect()

m.add_arm_handler(lambda arm, xdir: print('arm', arm, 'xdir', xdir))
m.add_pose_handler(lambda p: print('pose', p))
# m.add_imu_handler(lambda quat, acc, gyro: print('quaternion', quat))
m.sleep_mode(1)
m.set_leds([128, 128, 255], [128, 128, 255])  # purple logo and bar LEDs
m.vibrate(1)

try:
    while True:
        m.run(1)

        if HAVE_PYGAME:
            for ev in pygame.event.get():
                if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
                    raise KeyboardInterrupt()
                # elif ev.type == KEYDOWN and ev.unicode == 'd':
                #     m.disconnect()
                #     print("Disconnected")
                #     raise KeyboardInterrupt()
                elif ev.type == KEYDOWN:
                    if K_1 <= ev.key <= K_3:
                        m.vibrate(ev.key - K_0)
                    if K_KP1 <= ev.key <= K_KP3:
                        m.vibrate(ev.key - K_KP0)

except KeyboardInterrupt:
    pass
finally:
    # m.power_off()
    # print("Power off")
    m.disconnect()
    print("Disconnected")
    # command = raw_input("Do you want to (d)isconnect or (p)ower off?\n")
    # if command == 'd':
    #     m.disconnect()
    #     print("Disconnected")
    # elif command == 'p':
    #     m.power_off()
    #     print("Power off")
