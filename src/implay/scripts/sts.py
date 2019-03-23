#!/usr/bin/python
# Used to select a training set from the db
import numpy as np
from random import shuffle
from db import db

def select_training_set(nimgs):
    update_rewards()
    fdb = db("/caffe/ros/db/frames.db", False)
    training_set = []
    new_rollouts = fdb.list_used_rollouts(0)
    new_rollouts = [r[0] for r in new_rollouts]
    nlen = len(new_rollouts)
    # Find old rollouts that score higher than 2000
    old_rollouts = fdb.get_max_score_rollout()
    old_rollouts.sort(reverse=True)
    scores = [s for s, r in old_rollouts]
    best_rollouts = [r for s, r in old_rollouts if s > 2000]
    if nlen != 0:
        # If we have new rollouts mix in three times as many old
        old = np.random.choice(best_rollouts, nlen*3, replace=False).tolist()
        src_set = new_rollouts + old
        fdb.update_rollout_rec_used()
        fdb.commit()
    else:
        # Otherwise just use 4 of them
        src_set = np.random.choice(best_rollouts, 4, replace=False).tolist()
    print "sts: src_set", src_set
    print "sts: avg max score", round(np.mean(scores))
    with open("maxlog.txt", "a") as maxlog:
        maxlog.write("{0} {1}\n".format(src_set[0], round(np.mean(scores))))
    for r in src_set:
        rollout = fdb.get_rollout_fn_action_rew(r)
        if len(rollout) == 0:
            print "rollout", r, "is empty"
            continue
        old_fns = [fn for fn, a, rew in rollout[:nimgs-1]]
        for fn, action, reward in rollout[nimgs-1:-1]:
            if reward >= 0:
                training_set.append((old_fns + [fn], action))
            old_fns = old_fns[1:]
            old_fns.append(fn)
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
        frames = [frame for frame, score, action in rollout]
        ri_max = len(scores)
        ri0 = 1
        if ri0 >= ri_max:
            continue
        decay = 0.90
        ri = ri_max-1
        # last score is -1 if fail
        if scores[ri] >= 0:
            reward = 42.0
        else:
            reward = -300.0
        fdb.update_reward(int(reward), frames[ri], r)
        # skip over the block of identical scores at the end
        ri -= 1
        final_score = scores[ri]
        fdb.update_reward(int(reward), frames[ri], r)
        ri -= 1
        while scores[ri] == final_score:
            fdb.update_reward(int(reward), frames[ri], r)
            ri -= 1
        # do the rest
        last_fs = final_score
        while ri >= ri0:
            dfs = last_fs - scores[ri]
            reward = dfs + reward * decay
            # print r, ri, scores[ri], action, round(reward, 0)
            fdb.update_reward(int(round(reward)), frames[ri], r)
            tr += reward
            rc += 1
            last_fs = scores[ri]
            ri -= 1
        fdb.update_rollout_rec_reward(r)
    fdb.commit()

# Some test code
if __name__ == '__main__':
    print select_training_set(3)
