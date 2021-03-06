#!/usr/bin/python
import numpy as np
import cv2
import time
import os
import threading
import random

import rospy
import ros_numpy

from sensor_msgs.msg import Image
from implay.msg import DoobotActionArchive
from game.msg import PlayCtl
from deploy_tf import deploy
from sts import select_training_set

from db import db

# Note should be cols, rows for resize
small_size = (80, 128)

# These should match PlayCtl msgs
tf_state_startup = 0
tf_state_online = 1
tf_state_training = 2

# Number of images used in an inference
nimgs = 3

class tf_handler(object):
    def __init__(self, prefix):
        self.time_to_train = False
        self.state = tf_state_startup
        self.dn = deploy()
        self.dn.restore(prefix)
        self.batch_size = 10

    def process_imgs(self, imgs):
        fimgs = self.dn.format_images(imgs)
        values = self.dn.run(fimgs)
        return values[0]

    def make_batch(self, items, index):
        batch_x = []
        batch_y = []
        for i in range(self.batch_size):
            fns, l = items[index]
            index = (index + 1) % len(items)
            label = np.zeros(5, dtype = np.float)
            label[l] = 1.0
            batch_y.append(label)
            imgs = self.dn.load_images(fns)
            batch_x.append(imgs)
        batch_x = np.concatenate(batch_x, axis=0)
        batch_y = np.array(batch_y)
        return(batch_x, batch_y, index)

    def train(self):
        self.state = tf_state_training
        training_set = select_training_set(nimgs)
        index = 0
        nbatches = (len(training_set) + self.batch_size)//self.batch_size
        for i in range(nbatches):
            x, y, index = self.make_batch(training_set, index)
            loss, acc = self.dn.train(x, y)
            if i > 0 and i % 10 == 0:
                lp = round(loss, 3)
                ap = round(acc * 100, 1)
                print "Batch", i, "of", nbatches, "loss:", lp, "acc:", ap
        if i % 10 != 0:
            print "Batch", nbatches, "of", nbatches, "loss:", lp, "acc:", ap
        self.dn.save()
        print "Save done"
        self.state = tf_state_online
        self.time_to_train = False

    def system_up(self):
        self.state = tf_state_online
        
    def handle_play_ctl_msg(self, msg):
        # print "tfplay got a play_ctl msg"
        if msg.train == 1:
            self.time_to_train = True

class image_handler(object):
    def __init__(self):
        self.tlock = threading.Lock()
        self.cur_imgs = []
        self.have_imgs = False 

    def acquire_image(self, msg):
        global ct
        if msg.header.seq % 3 != 0:
            return
        if len(self.cur_imgs) == nimgs:
            print "tfplay frame dropped", msg.header.seq
            self.have_imgs = False
            self.cur_imgs = []
            return
        self.tlock.acquire(True)
        img = ros_numpy.numpify(msg)
        img = img[...,::-1]
        img = cv2.resize(img, small_size)
        self.cur_imgs.append(img)
        self.cur_frame = msg.header.seq
        if len(self.cur_imgs) == nimgs:
            self.have_imgs = True
        self.tlock.release()

    def retrieve_image(self):
        self.tlock.acquire(True)
        rval = (self.cur_frame, list(self.cur_imgs))
        self.cur_imgs.pop(0)
        self.have_imgs = False
        self.tlock.release()
        return rval

def send_move(pub, action, action_src, frame):
    move_msg = DoobotActionArchive()
    move_msg.action = action
    move_msg.action_src = action_src
    move_msg.fseq = frame
    pub.publish(move_msg)

def send_play_ctl(ctl_pub, cur_state):
    play_ctl_msg = PlayCtl()
    play_ctl_msg.header.stamp = rospy.Time.now()
    play_ctl_msg.tf_state = cur_state
    play_ctl_msg.msg_src = 1
    ctl_pub.publish(play_ctl_msg)

def image_callback(msg, args):
    ih = args[0]
    tfh = args[1]
    if tfh.state != tf_state_online:
        return
    ih.acquire_image(msg)

def play_ctl_callback(msg, args):
    tfh = args[0]
    if msg.msg_src == 1: # Ignore my msgs
        return
    tfh.handle_play_ctl_msg(msg)

def softmax(v):
    x = np.array(v, dtype=np.float64)
    xe = np.exp(x)
    return xe / xe.sum()

def main():
    # Ros init, setup image msg thread
    rospy.init_node("tfplay", anonymous = True)
    tfh = tf_handler("/caffe/ros/src/implay/scripts/snapshots/net2a")
    ih = image_handler()
    pub = rospy.Publisher('implay/DoobotActionArchive', DoobotActionArchive, queue_size = 1)
    ctl_pub = rospy.Publisher('game/PlayCtl', PlayCtl, queue_size = 1)
    rospy.Subscriber("camera1/image_raw", Image, image_callback, (ih, tfh), queue_size=1,
                         buff_size=512*320*3+1000)
    rospy.Subscriber("game/PlayCtl", PlayCtl, play_ctl_callback, (tfh, ))
    tfh.system_up()
    send_play_ctl(ctl_pub, tf_state_online)
    eps = 0.5
    while not rospy.is_shutdown():
        if ih.have_imgs:
            frame, imgs = ih.retrieve_image()
            actions = tfh.process_imgs(imgs)
            dist = softmax(actions) 
            # pdist = [round(v*100, 1) for v in dist]
            # print "tfplay: dist", pdist
            if random.random() < eps:
                action = np.argmax(actions)
                action_src = 1
            else:
                action = np.random.choice(len(dist), 1, p=dist)[0]
                action_src = 2
            send_move(pub, action, action_src, frame)
        if tfh.time_to_train:
            send_play_ctl(ctl_pub, tf_state_training)
            tfh.train()
            send_play_ctl(ctl_pub, tf_state_online)
        time.sleep(1.0/100.0)

if __name__ == '__main__':
    print "Starting tf_play"
    main()
                          
