#!/usr/bin/python
# Try to do some training
# Pushes the whole database through 

import os
import sys
import math
import numpy as np
import time
from random import shuffle

from deploy_tf import deploy
from db import db

nimgs = 3
def build_train():
    all_images = []
    fdb = db("/caffe/ros/db/frames.db", False)
    rollouts = fdb.list_rollouts()
    for (r, ) in rollouts:
        rollout = fdb.get_rollout_fn_action_rew(r)
        old_fns = [fn for fn, action, rew in rollout[:nimgs-1]]
        for fn, action, reward in rollout[nimgs-1:-1]:
            if reward >= 0:
                all_images.append((old_fns + [fn], action))
            old_fns = old_fns[1:]
            old_fns.append(fn)
    shuffle(all_images)
    return all_images

def make_batch(dn, items, n, index):
    batch_x = []
    batch_y = []
    for i in range(n):
        fns, l = items[index]
        index = (index + 1) % len(items) 
        label = np.zeros(5, dtype = np.float)
        label[l] = 1.0
        batch_y.append(label)
        imgs = dn.load_images(fns)
        batch_x.append(imgs)
    batch_x = np.concatenate(batch_x, axis=0)
    batch_y = np.array(batch_y)
    return(batch_x, batch_y, index)

if __name__ == '__main__':

    items = build_train()

    losses = []
    accuracies = []
    times = []
    dn = deploy()
    dn.restore("snapshots/net2a")
    index = 0
    batch_size = 10
    nbatches = len(items) // batch_size
    for i in range(1, nbatches+1):
        batch_x, batch_y, index = make_batch(dn, items, batch_size, index)
        start = time.time()
        loss, acc = dn.train(batch_x, batch_y) 
        times.append(time.time()-start)
        acc *= 100.0
        fmt_str = "Batch: {} of {}, Loss: {:.3f}, Accuracy: {:3.0f}%"
        print fmt_str.format(i, nbatches, loss, acc)
    dn.save()
