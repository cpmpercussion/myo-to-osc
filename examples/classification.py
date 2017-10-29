#
# Original work Copyright (c) 2014 Danny Zhu
# Modified work Copyright (c) 2017 Matthias Gazzari
# 
# Licensed under the MIT license. See the LICENSE file for details.
#

from collections import Counter, deque
import sys
import struct
import numpy as np
from myo_raw import MyoRaw
try:
    from sklearn import neighbors, svm
    HAVE_SK = True
except ImportError:
    HAVE_SK = False
try:
    import pygame
    from pygame.locals import *
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False

SUBSAMPLE = 3
K = 15

class NNClassifier(object):
    '''A wrapper for sklearn's nearest-neighbor classifier that stores
    training data in vals0, ..., vals9.dat.'''

    def __init__(self):
        for i in range(10):
            with open('vals%d.dat' % i, 'ab') as f: pass
        self.read_data()

    def store_data(self, cls, vals):
        with open('vals%d.dat' % cls, 'ab') as f:
            f.write(struct.pack('<8H', *vals))

        self.train(np.vstack([self.X, vals]), np.hstack([self.Y, [cls]]))

    def read_data(self):
        X = []
        Y = []
        for i in range(10):
            X.append(np.fromfile('vals%d.dat' % i, dtype=np.uint16).reshape((-1, 8)))
            Y.append(i + np.zeros(X[-1].shape[0]))

        self.train(np.vstack(X), np.hstack(Y))

    def train(self, X, Y):
        self.X = X
        self.Y = Y
        if HAVE_SK and self.X.shape[0] >= K * SUBSAMPLE:
            self.nn = neighbors.KNeighborsClassifier(n_neighbors=K, algorithm='kd_tree')
            self.nn.fit(self.X[::SUBSAMPLE], self.Y[::SUBSAMPLE])
        else:
            self.nn = None

    def nearest(self, d):
        dists = ((self.X - d)**2).sum(1)
        ind = dists.argmin()
        return self.Y[ind]

    def classify(self, d):
        if self.X.shape[0] < K * SUBSAMPLE: return 0
        if not HAVE_SK: return self.nearest(d)
        return int(self.nn.predict(d)[0])


class Myo(MyoRaw):
    '''Adds higher-level pose classification and handling onto MyoRaw.'''

    HIST_LEN = 25

    def __init__(self, cls, tty=None):
        MyoRaw.__init__(self, tty)
        self.cls = cls
        self.history = deque([0] * Myo.HIST_LEN, Myo.HIST_LEN)
        self.history_cnt = Counter(self.history)
        self.add_emg_handler(self.emg_handler)
        self.last_pose = None
        self.pose_handlers = []

    def emg_handler(self, emg, moving):
        y = self.cls.classify(emg)
        self.history_cnt[self.history[0]] -= 1
        self.history_cnt[y] += 1
        self.history.append(y)
        r, n = self.history_cnt.most_common(1)[0]
        if self.last_pose is None or (n > self.history_cnt[self.last_pose] + 5 and n > Myo.HIST_LEN / 2):
            self.on_raw_pose(r)
            self.last_pose = r

    def add_raw_pose_handler(self, h):
        self.pose_handlers.append(h)

    def on_raw_pose(self, pose):
        for h in self.pose_handlers:
            h(pose)

class EMGHandler(object):
    def __init__(self, m):
        self.recording = -1
        self.m = m
        self.emg = (0,) * 8

    def __call__(self, emg, moving):
        self.emg = emg
        if self.recording >= 0:
            self.m.cls.store_data(self.recording, emg)

def classify(m):
    if HAVE_PYGAME:
        pygame.init()
        w, h = 800, 320
        scr = pygame.display.set_mode((w, h))
        font = pygame.font.Font(None, 30)

    hnd = EMGHandler(m)
    m.add_emg_handler(hnd)
    m.connect(filtered=True)

    while True:
        m.run()
        r = m.history_cnt.most_common(1)[0][0]
        if HAVE_PYGAME:
            for ev in pygame.event.get():
                if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
                    raise KeyboardInterrupt()
                elif ev.type == KEYDOWN:
                    if K_0 <= ev.key <= K_9:
                        hnd.recording = ev.key - K_0
                    elif K_KP0 <= ev.key <= K_KP9:
                        hnd.recording = ev.key - K_Kp0
                    elif ev.unicode == 'r':
                        hnd.cl.read_data()
                elif ev.type == KEYUP:
                    if K_0 <= ev.key <= K_9 or K_KP0 <= ev.key <= K_KP9:
                        hnd.recording = -1

            scr.fill((0, 0, 0), (0, 0, w, h))

            for i in range(10):
                x = 0
                y = 0 + 30 * i
                clr = (0,200,0) if i == r else (255,255,255)
                txt = font.render('%5d' % (m.cls.Y == i).sum(), True, (255,255,255))
                scr.blit(txt, (x + 20, y))
                txt = font.render('%d' % i, True, clr)
                scr.blit(txt, (x + 110, y))
                scr.fill((0,0,0), (x+130, y + txt.get_height() / 2 - 10, len(m.history) * 20, 20))
                scr.fill(clr, (x+130, y + txt.get_height() / 2 - 10, m.history_cnt[i] * 20, 20))

            if HAVE_SK and m.cls.nn is not None:
                dists, inds = m.cls.nn.kneighbors(hnd.emg)
                for i, (d, ind) in enumerate(zip(dists[0], inds[0])):
                    y = m.cls.Y[SUBSAMPLE*ind]
                    pos = (650, 20 * i)
                    txt = '%d %6d' % (y, d)
                    clr = (255, 255, 255)
                    scr.blit(font.render(txt, True, clr), pos)

            pygame.display.flip()
        else:
            for i in range(10):
                if i == r: sys.stdout.write('\x1b[32m')
                print(i, '-' * m.history_cnt[i], '\x1b[K')
                if i == r: sys.stdout.write('\x1b[m')
            sys.stdout.write('\x1b[11A')
            print()

def detect(m):
    import subprocess
    m.add_raw_pose_handler(print)

    def page(pose):
        if pose == 5:
            subprocess.call(['xte', 'key Page_Down'])
        elif pose == 6:
            subprocess.call(['xte', 'key Page_Up'])

    m.add_raw_pose_handler(page)
    m.connect()
    while True:
        m.run()

if __name__ == '__main__':
    m = Myo(NNClassifier(), sys.argv[1] if len(sys.argv) >= 2 else None)
    try:
        while True:
            choice = input('Do you want to (c)lassify or (d)etect poses?\n')
            if choice == 'c':
                classify(m)
                break
            elif choice == 'd':
                detect(m)
                break
    except KeyboardInterrupt:
        pass
    finally:
        m.disconnect()
        print("Disconnected")
