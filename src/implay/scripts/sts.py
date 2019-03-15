#!/usr/bin/python
# Used to select a training set from the db
import numpy as np
from random import shuffle
from db import db

def select_training_set():
    update_rewards()
    fdb = db("/caffe/ros/db/frames.db", False)
    training_set = []
    new_rollouts = fdb.list_used_rollouts(0)
    new_rollouts = [v[0] for v in new_rollouts]
    nlen = len(new_rollouts)
    # Find the top 30 highest scoring old rollouts
    old_rollouts = fdb.get_max_score_rollout()
    scores = [v[0] for v in old_rollouts]
    old_rollouts.sort(reverse=True)
    old_rollouts = [v[1] for v in old_rollouts]
    old_rollouts = old_rollouts[:30]
    shuffle(old_rollouts)
    if nlen != 0:
        # If we have new rollouts mix in three times as many old
        src_set = new_rollouts + old_rollouts[:nlen*3]
        fdb.update_rollout_rec_used()
        fdb.commit()
    else:
        # Otherwise just use 4 of them
        src_set = old_rollouts[:4]
    print "sts: src_set", src_set
    print "sts: avg max score", round(np.mean(scores))
    with open("maxlog.txt", "a") as maxlog:
        maxlog.write("{0} {1}\n".format(new_rollouts[0], round(np.mean(scores))))
    for r in src_set:
        rollout = fdb.get_rollout_fn_action_rew(r)
        if len(rollout) == 0:
            print "rollout", r, "is empty"
            continue
        last_fn, last_action, last_rew = rollout[0]
        for fn, action, reward in rollout[1:-1]:
            if reward >= 0:
                training_set.append((last_fn, fn, action))
            last_fn = fn
    shuffle(training_set)
    return training_set

def update_rewards(all_rollouts=False):
    fdb = db("/caffe/ros/db/frames.db", False)
    fn_idx = 1
    if all_rollouts:
        rollouts = fdb.list_rollouts()
    else:
        rollouts = fdb.list_unrewarded_rollouts()
    tr = 0.0
    rc = 0
    for (r, ) in rollouts:
        rollout = fdb.get_rollout_frame_score_action(r)
        scores = [score for frame, score, action in rollout]
        ri_max = len(scores)
        ri0 = 1
        if ri0 >= ri_max:
            continue
        ri = ri_max-1
        decay = 0.90
        fs = scores[ri]
        last_fs = fs
        frame, score, action = rollout[ri]
        # last score is -1 if fail
        if score >= 0:
            reward = 42.0
        else:
            reward = -40.0
            fdb.update_reward(int(round(reward)), frame, r)
        skip_to_action = True
        while ri >= ri0:
            ri -= 1
            fs = scores[ri]
            frame, score, action = rollout[ri]
            if skip_to_action:
                skip_to_action = action == 0
            else:
                dfs = last_fs - fs
                reward = dfs + reward * decay
            # print r, ri, fs, action, round(reward, 0)
            fdb.update_reward(int(round(reward)), frame, r)
            tr += reward
            rc += 1
            last_fs = fs
        fdb.update_rollout_rec_reward(r)
    fdb.commit()

# Some test code
if __name__ == '__main__':
    print select_training_set()
