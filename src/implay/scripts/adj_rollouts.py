#!/usr/bin/python
# Used to go back through all of the rewards in
# the database when the reward function has changed
from sts import update_rewards

update_rewards(all_rollouts=True)
