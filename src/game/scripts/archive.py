#!/usr/bin/python
import numpy as np
import cv2
import curses
import time
import os
import gc
import threading
from subprocess import call

import rospy
import ros_numpy

from sensor_msgs.msg import Image
from score.msg import DoodleScore 
from game.msg import DoobotAction
from game.msg import PlayCtl
from implay.msg import DoobotActionArchive

from db import db
from ctimers import ctimer
tick_dt = 1.0/100.0
ct = ctimer()
# Note should be cols, rows for resize
small_size = (80, 128)

# Misc graphics things

def get_grv(image, r, c):
    (rows, cols, colors) = image.shape
    if colors == 3:
        (bl, gr, rd) = image[r, c]
        l = float(gr) * 0.587 + float(rd) * 0.299 + float(bl) * 0.114
    else:
        l = float(image[r, c])
    return l

def count_dark_pix(image, limit, r):
    for c in range(limit):
        if get_grv(image, r, c) > 100:
            break
    return c

# Database can't keep up 
class db_cache(db):
    def __init__(self, db_name, do_init):
        super(db_cache, self).__init__(db_name, do_init)
        self.records = []
        self.actions = []
        self.scores = []
        self.have_data = False
        self.tlock = threading.Lock()

    def save_record(self, frame_num, full_path):
        self.records.append((frame_num, full_path))
        self.have_data = True

    def update_action(self, action, frame_num):
        self.tlock.acquire(True)
        self.actions.append((action, frame_num))
        self.have_data = True
        self.tlock.release()

    def update_score(self, score, frame_num):
        self.scores.append((score, frame_num))
        self.have_data = True

    def commit(self):
        if not self.have_data:
            return
        for fn, p in self.records:
            super(db_cache, self).save_record(fn, p)
        self.records = []
        for a, fn in self.actions:
            super(db_cache, self).update_action(a, fn)
        self.actions = []
        for s, fn in self.scores:
            super(db_cache, self).update_score(s, fn)
        self.scores = []
        self.have_data = False
        super(db_cache, self).commit()

class image_handler(object):
    def __init__(self):
        self.tlock = threading.Lock()
        self.have_img = False
        self.at_end = True

    def acquire_image(self, msg, gs):
        global ct
        ct.log('start')
        if self.have_img:
            if gs.recording():
                print "Archive frame dropped", msg.header.seq
            return
        self.tlock.acquire(True)
        ct.log("got_lock")
        self.frame_num = msg.header.seq
        self.img = ros_numpy.numpify(msg)
        ct.log("numpify")
        self.have_img = True
        self.tlock.release()
        self.chk_end()
        cv2.imshow("chk_end", self.img)
        keycode = cv2.waitKey(20)
        keycode &= 0xff
        ct.log("imshow")
        if keycode != 255:
            gs.save_key(keycode, self.frame_num)
        ct.log("done")

    def retrieve_image(self):
        self.tlock.acquire(True)
        fn = self.frame_num
        img_copy = self.img.copy()
        self.tlock.release()
        return fn, img_copy

    def have_image(self):
        return self.have_img

    def image_processed(self):
        self.tlock.acquire(True)
        self.have_img = False
        self.tlock.release()

    def chk_end(self):
        grass1 = self.img[442:460, 90:100]
        grass2 = self.img[442:460, 230:240]
        tot1 = grass1.sum()
        tot2 = grass2.sum()
        self.at_end = tot1 < 10000 or tot2 < 10000

    def is_end(self):
        return self.at_end

# bot actions
actions = {'nothing': 0,
           'left': 1,
           'right': 2,
           'fire': 3,
           'start': 4,
           'fix_solenoid': 5,
          }

enables = {'enable': 1,
           'disable': 2,
           'single_cmd': 3,
          }

# controls
start_recording = -1
stop_recording = -2
quit = -3
start_training = -4

key2action = {
            ord('w'): actions['fire'],
            ord('a'): actions['left'],
            ord('d'): actions['right'],
            ord('s'): actions['start'],
            ord('f'): actions['fix_solenoid'],
            ord('i'): start_recording,
            ord('t'): stop_recording,
            ord('g'): start_training,
            ord('q'): quit,
          }

class global_state(object):
    def __init__(self, cdb, ih, pub, ctl_pub):
        self.cdb = cdb
        self.ih = ih
        self.pub = pub
        self.ctl_pub = ctl_pub
        self.rec = False
        self.tlock = threading.Lock()
        self.have_action = False
        self.have_score = False
        self.active = False
        self.prefix="/caffe/ros/db/r{:05d}/"
        self.fn="f{:05d}.jpg"
        self.frame_count = 0
        self.skip = 3
        self.last_frame = 0
        self.sframe = 0
        self.last_img = None
        self.recenter = False
        self.recenter_count = 0
        self.recenter_frame = 0
        self.moves = 0
        self.do_recenter()
        self.imwrite_cache = []
        self.tlock = threading.Lock()
        self.supress_keys = 0
        self.last_train_state = -1 # unknown
        self.training_running = False
        self.play_ctl_msg_received = False

    def send_action(self, action, enable=enables['enable']):
        self.tlock.acquire(True)
        global ct
        ct.log('start')
        if action == actions['left']:
            if self.moves == -7:
                self.tlock.release()
                return
            self.moves -= 1
        if action == actions['right']:
            if self.moves == 7:
                self.tlock.release()
                return
            self.moves += 1
        action_msg = DoobotAction()
        action_msg.header.stamp = rospy.Time.now()
        action_msg.action = action
        action_msg.enable = enable
        self.pub.publish(action_msg)
        ct.log('done')
        self.tlock.release()

    def send_train_start(self):
        self.send_play_ctl(1)

    def save_key(self, keycode, frame):
        global ct
        ct.log('start')
        if not keycode in key2action:
            print "bad key:", chr(keycode)
            return
        action = key2action[keycode]
        if action >= 0:
            print action
            self.send_action(action)
            self.supress_keys = 3
            self.cdb.update_action(action, frame/self.skip)
        elif action == start_recording:
            # self.send_action(actions['start'], enable=enables['enable'])
            self.rec = True
            self.frame_count = 0
        elif action == stop_recording:
            self.rec = False
            self.send_action(actions['nothing'], enable=enables['disable'])
        elif action == start_training:
            self.send_train_start()
        elif action == quit:
            # SIGINT == 2.
            os.kill(os.getppid(), 2)
        ct.log('done')

    def recording(self):
        return self.rec

    def save_score(self, score, frame):
        self.cdb.update_score(score, frame/self.skip)

    def save_implay_action(self, action, frame):
        if self.supress_keys > 0:
            self.supress_keys -= 1
        else:
            self.send_action(action)
            self.cdb.update_action(action, frame/self.skip)

    def do_recenter(self):
        global ct
        ct.log('start')
        if not self.recenter:
            self.recenter = True
        if self.last_img is None:
            return
        if self.recenter_count != 0:
            self.recenter_count -= 1
            return
        if self.recenter_frame == self.last_frame:
            return
        dtop = count_dark_pix(self.last_img, 100, 120)
        dbot = count_dark_pix(self.last_img, 100, 370)
        if abs(dtop - dbot) < 5:
            self.recenter = False
            self.moves = 0
            ct.log('done')
            return
        if dtop < dbot:
            self.send_action(actions['right'], enable=enables['single_cmd'])
        else:
            self.send_action(actions['left'], enable=enables['single_cmd'])
        self.recenter_frame = self.last_frame
        self.recenter_count = int(0.2/tick_dt)

    def cache_imwrite(self, path, img, flags):
        self.imwrite_cache.append((path, img, flags))

    def send_play_ctl(self, training_msg):
        play_ctl_msg = PlayCtl()
        play_ctl_msg.header.stamp = rospy.Time.now()
        play_ctl_msg.train = training_msg
        play_ctl_msg.msg_src = 0
        self.ctl_pub.publish(play_ctl_msg)

    def handle_play_ctl(self, msg):
        if msg.msg_src == 0: # ignore my msgs
            return
        self.last_train_state = msg.tf_state
        self.play_ctl_msg_received = True

    def tick(self):
        global ct
        ct.log('start')
        if self.ih.have_image():
            self.last_frame, self.last_img = self.ih.retrieve_image()
            self.sframe = self.last_frame/self.skip
            self.ih.image_processed();
            ct.log('retrieve')
            if self.rec and self.last_frame % self.skip == 0:
                rollout = self.cdb.get_rollout()
                path = self.prefix.format(rollout)
                if not os.path.isdir(path):
                    os.makedirs(path)
                fname = self.fn.format(self.sframe)
                full_path = path + fname
                self.small_img = cv2.resize(self.last_img, small_size)
                self.cache_imwrite(full_path, self.small_img,
                                   [cv2.IMWRITE_JPEG_QUALITY, 100])
                self.cdb.save_record(self.sframe, full_path)
                self.frame_count += 1
                ct.log('imwrite')
        if self.recenter:
            self.do_recenter()
        if self.play_ctl_msg_received:
            self.play_ctl_msg_received = False
            if self.last_train_state == 2:
                print "Training started"
                self.training_running = True
            elif self.last_train_state == 1:
                print "Training done"
                self.training_running = False
                self.rec = True
                self.frame_count = 0
                self.send_action(actions['start'])
                self.cdb.update_action(actions['start'], self.last_frame/self.skip)
        if (self.active and self.ih.is_end()) or self.frame_count == 1500:
            self.active = False
            self.rec = False
            self.send_action(actions['nothing'], enable=enables['disable'])
            if self.ih.is_end():
                self.cdb.update_score(-1, self.sframe)
            ct.log('commit start')
            if self.ih.is_end():
                print "Detected end at", self.frame_count
            else:
                print "Max frames stored:", self.frame_count
            self.frame_count = 0
            self.cdb.insert_rollout_rec()
            self.cdb.commit()
            ct.log('commit cdb')
            for path, img, flags in self.imwrite_cache:
                cv2.imwrite(path, img, flags)
            self.imwrite_cache = []
            ct.log('commit imwrites')
            call("sync")  # Push buffered data to disk
            ct.log('commit sync')
            gc.collect()  # Garbage collection
            ct.log('commit collect')
            self.cdb.advance_rollout()
            self.do_recenter()
            ct.dump()
            ct.clear()
            call("sync")
            print "Commit done"
            self.send_train_start()
            self.training_running = True
        elif not self.active and not self.ih.is_end() and not self.training_running:
            self.active = True

def image_callback(msg, args):
    ih = args[0]
    gs = args[1]
    ih.acquire_image(msg, gs)

def score_callback(msg, args):
    gs = args[0]
    if gs.recording():
        gs.save_score(msg.score, msg.fseq)
        
def action_callback(msg, args):
    gs = args[0]
    if gs.recording():
        action = msg.action
        if msg.action_src == 1:
            print "        ", action
        else:
            print "    ", action
        gs.save_implay_action(action, msg.fseq)

def play_ctl_callback(msg, args):
    gs = args[0]
    gs.handle_play_ctl(msg)

def main():
    # Ros init, setup image msg thread
    call("sync")
    rospy.init_node("game", anonymous = True)
    ih = image_handler()
    cdb = db_cache("/caffe/ros/db/frames.db", False)
    pub = rospy.Publisher('game/DoobotAction', DoobotAction, queue_size = 1)
    ctl_pub = rospy.Publisher('game/PlayCtl', PlayCtl, queue_size = 1)
    gs = global_state(cdb, ih, pub, ctl_pub)
    rospy.Subscriber("camera1/image_raw", Image, image_callback, (ih, gs), queue_size=1,
                         buff_size=512*320*3+1000)
    rospy.Subscriber("score/DoodleScore", DoodleScore, score_callback, (gs,))
    rospy.Subscriber("implay/DoobotActionArchive", DoobotActionArchive, action_callback, (gs,))
    rospy.Subscriber("game/PlayCtl", PlayCtl, play_ctl_callback, (gs, ))
    while not rospy.is_shutdown():
        gs.tick()
        time.sleep(tick_dt)

if __name__ == '__main__':
    print "Starting Archive"
    main()
