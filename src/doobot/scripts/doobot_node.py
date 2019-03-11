#!/usr/bin/python
import glob
import threading
import rospy

from game.msg import DoobotAction
from doobot_motor import motor

def find_port(motor):
    port_count=0
    devices = glob.glob("/dev/ttyACM*")
    if len(devices) == 1:
        motor.set_port(devices[0])
    elif port_count == 0:
        print "no tty ports found!"
        print "check for /dev/ttyACM*"
        exit(1)
    else:
        print "too many USB ports!"

class cbq(object):
    def __init__(self, motor):
        self.enabled = False
        self.queue = []
        self.motor = motor
        self.tlock = threading.Lock()
        self.in_callback = False

    def callback(self, msg):
        self.tlock.acquire(True)
        if len(self.queue) > 5:
            print "doobot dropping", msg.action
            self.tlock.release()
            return
        self.queue.append(msg)
        if self.in_callback:
            self.tlock.release()
            return
        self.in_callback = True
        self.tlock.release()
        while True:
            self.tlock.acquire(True)
            if len(self.queue) == 0:
                self.in_callback = False
                self.tlock.release()
                return
            msg = self.queue.pop(0)
            self.tlock.release()
            # 0 means do nothing
            if msg.enable & 1:
                self.enabled = True
            if self.enabled:
                if msg.action == 1:
                    self.motor.sm_cmd(20, 1, -5)
                elif msg.action == 2:
                    self.motor.sm_cmd(20, 1, 5)
                elif msg.action == 3:
                    self.motor.pulse_solenoid()
                elif msg.action == 4:
                    self.motor.run_servo()
                elif msg.action == 5:
                    self.motor.fix_solenoid()
            if msg.enable & 2:
                self.enabled = False

def bot_callback(msg, args):
    cbq = args[0]
    cbq.callback(msg)

if __name__=='__main__':
    motor = motor()
    find_port(motor)
    rospy.init_node("doobot", anonymous = True)
    my_cbq = cbq(motor)
    rospy.Subscriber("game/DoobotAction", DoobotAction, bot_callback, (my_cbq,))
    rospy.spin()
    motor.motors_off()
