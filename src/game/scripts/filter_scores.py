#!/usr/bin/python
# pots /caffe/ros/db/frames.db

import sys
import numpy as np
from db import db

def find_0(scores):
    ri = 0
    ri_max = len(scores)
    score = 1
    while ri < ri_max and scores[ri] != 0:
        ri += 1
    return ri

def skip_0(scores, riz):
    ri = riz
    ri_max = len(scores)
    while ri < ri_max and scores[ri] == 0:
        ri += 1
    return ri

def do_filter(x):
    dtc_max = 168
    y = []
    y.append(x[0])
    last_dt = 0.0
    for i in range(1, len(x)):
        dt = x[i] - y[i-1]
        if (dt < 0.0):
            y.append(y[i-1])
        else:
            if abs(dt - last_dt) > dtc_max:
                y.append(y[i-1])
            else:
                y.append(x[i])
                last_dt = dt
    last = len(x)-1
    if x[last] == -1:
        y[last] = -1
    return y

# Takes an array of (score, frame) entries
# Returns another
def filter_scores(a):
    scores = [s for s, f in a]
    frames = [f for s, f in a]
    ri = find_0(scores)
    if ri == len(scores):
        return a
    for i in range(ri):
        scores[i] = 0
    scores = do_filter(scores)
    return zip(scores, frames)

# Call standalone to fix the db
if __name__ == '__main__':
    fdb = db("/caffe/ros/db/frames.db", False)
    if len(sys.argv) == 2:
        rollouts = [(int(sys.argv[1]),)]
    else:
        rollouts = fdb.list_rollouts()
    for (r, ) in rollouts:
        rollout = fdb.get_rollout_frame_score(r)
        ri = 0
        # Swizzle to match filter_scores()
        a = [(s, f) for f, s in rollout]
        s0, f0 = a[0]
        b = filter_scores(a)
        for i in range(len(a)):
            sa, fa = a[i]
            sb, fb = b[i]
            if (sa != sb):
                fdb.update_score(sb, fb, r)
                print r, fa-f0, sa, '->', sb
    fdb.commit()
