import sys
import rospy

class ctimer(object):
    def __init__(self):
        self.timers = {}

    def log(self, name):
        caller_frame = sys._getframe(1)
        caller_name = caller_frame.f_code.co_name
        timer_name = caller_name
        if timer_name not in self.timers:
            self.timers[timer_name] = []
        self.timers[timer_name].append((name, rospy.get_time()))

    def clear(self):
        self.timers = {}

    def dump(self):
        with open("/caffe/ros/ctimer_log.txt", "a") as f:
            for tname, tv in self.timers.items():
                t0 = 0
                lt = 0
                for n, t in tv:
                    if n == 'start':
                        t0 = t
                    else:
                        f.write("{} {} {:.6f}, {:.6f}\n".format(tname, n, t-lt, t-t0))
                    lt = t
