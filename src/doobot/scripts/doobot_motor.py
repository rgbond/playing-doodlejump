#!/usr/bin/python

from Tkinter import *
import serial
import glob
import argparse
import time

# Knows about the eibot on the end of the USB cable
class motor:
    def __init__(self, tty_name=None, verbose=0):
        self.spr = 30
        self.verbose = verbose
        self.tty_name = tty_name
        self.cur_tty = None
        self.m1_cur_pos = 0
        self.m2_cur_pos = 0
        self.m1_last_dir = 1
        self.m2_last_dir = 1
        self.laser_on = False
        self.port = serial.Serial(self.tty_name)
        self.cur_servo_min = -1
        self.cur_servo_max = -1
        self.params = {
                       'm1_min_pos'    : -self.spr/2,
                       'm1_max_pos'    : self.spr/2,
                       'm1_mode'       : 3,
                       'steps_per_sec' : 800,
                       'acceleration'  : 2800,
                       'accel_deltat'  : 20,
                       'servo_min'     : 5000,
                       'servo_max'     : 15400,
                       'servo_pause'   : 700,
                       'solenoid_rate' : 125,
                      }
        if tty_name != None:
            self.set_port(tty_name)

    def set_port(self, tty_name):
        if self.verbose > 0:
            print "motor.set_port:", tty_name
        if tty_name != self.tty_name:
            if self.port.isOpen:
                self.port.close()
            self.tty_name = tty_name
            self.port.port = self.tty_name
            self.port.open()
            if self.verbose > 0:
                print "motor.set_port opened", tty_name
        if not self.port.isOpen():
            print "motor can't open", self.tty_name
            return False
        return True

    def reopen_port(self, tty_name):
        if self.verbose > 0:
            print "motor.reopen_port:", self.tty_name
        self.port.close()
        self.port.open()
        if not self.port.isOpen():
            print "motor can't open", self.tty_name

    def send(self, s):
        if self.verbose > 0:
            print "send: port", self.tty_name, "str", repr(s)
        self.port.write(s)

    def get_chars(self, n):
        val = ""
        for i in range(n):
            ch = self.port.read(1)
            if self.verbose > 1:
                print "get_chars:", ch, hex(ord(ch))
            val += ch
        if self.verbose > 0:
            print "get_chars:", repr(val)
        return val

    def dump_ch(self, msg, ch):
        v = hex(ord(ch))
        if ch < ' ' or ch > '~':
            ch = '.'
        print msg, ch, v

    def chk_reply(self, s):
        for i in range(len(s)):
            ch = self.port.read(1)
            if self.verbose > 1:
                self.dump_ch("chk_reply:", ch)
            if (ch != s[i]):
                self.dump_ch("chk_reply: invalid response", ch)
                while ord(ch) != 0x0a and self.port.inWaiting() > 0:
                    # inWaiting lies sometimes...
                    try:
                        ch = self.port.read(1)
                        self.dump_ch("chk_reply: discarding", ch)
                    except:
                        # nuclear option...
                        print "read aborted, reopening port"
                        self.reopen_port()
                return False
        if self.verbose > 0:
            print "chk_reply matched", repr(s)
        return True

    def sc_cmd(self, code, value):
        self.send_ok('SC,{0},{1}\r'.format(code, value))

    def send_ok(self, s):
        retry_count = 0
        while retry_count < 3:
            self.send(s)
            if self.chk_reply('OK\r\n'):
                return
            print "send_ok: retry", s
            retry_count += 1

    def steps_to_ms(self, steps):
        ms = int(float(abs(steps))*1000.0/float(self.params['steps_per_sec']) + 0.5)
        return ms

    def format_sm_cmd(self, ms, mx, steps):
        if mx == 1:
            cmd = 'SM,{0},{1},{2}\r'.format(ms, steps, 0)
        else:
            cmd = 'SM,{0},{1},{2}\r'.format(ms, 0, steps)
        return cmd

    def sm_cmd(self, ms, mx, steps):
        if steps == 0:
            return
        m1_mode = self.params['m1_mode']
        self.send_ok('EM,{0},{0}\r'.format(m1_mode, m1_mode))
        self.send_ok(self.format_sm_cmd(ms, mx, steps))
        m1_mode = self.params['m1_mode']

    def sm2_cmd(self, ms, m1_steps, m2_steps):
        if (m1_steps == 0) and (m2_steps == 0):
            return
        m1_mode = self.params['m1_mode']
        self.send_ok('EM,{0},{0}\r'.format(m1_mode, 0))
        self.send_ok('SM,{0},{1},{2}\r'.format(ms, m1_steps, m2_steps))

    def motors_off(self):
        self.send_ok('EM,0,0\r')

    def sp_cmd(self, param):
        self.send_ok("SP,{0}\r".format(param))

    def ac_cmd(self, an):
        self.send_ok("AC,{0},1\r".format(an))
        
    def get_analog(self):
        self.ac_cmd(1)
        self.ac_cmd(2)
        self.ac_cmd(3)
        self.send("A,\r")
        self.chk_reply('A,00:')
        self.get_chars(4)
        self.chk_reply(',01:')
        v1 = self.get_chars(4)
        self.chk_reply(',02:')
        v2 = self.get_chars(4)
        self.chk_reply(',03:')
        v3 = self.get_chars(4)
        # get ",11:xxxx"
        self.get_chars(8)
        self.chk_reply('\r\n')
        return (v1, v2, v3)

    def reset_counts(self):
        self.m1_cur_pos = 0
        self.m2_cur_pos = 0

    def make_ramp(self, steps):
        target = abs(steps)
        accel = self.params['acceleration']
        msdt = self.params['accel_deltat']
        dt = float(msdt)/1000.0
        max_steps_per_sec = self.params['steps_per_sec']
        ramp = []
        interval = 0
        total_steps = 0
        done = False
        while not done:
            interval += 1
            tot_time = interval * dt
            steps_per_sec = tot_time * accel
            cur_steps = int(steps_per_sec * dt + 0.5)
            if cur_steps * 2 + total_steps >= target:
                extra_steps = target - total_steps
                done = True
            elif steps_per_sec >= max_steps_per_sec:
                cur_steps = int(max_steps_per_sec * dt + 0.5)
                extra_steps = target - total_steps
                done = True
            else:
                ramp.append(cur_steps)
                total_steps += cur_steps * 2
        move = [s for s in ramp]
        while extra_steps > 0:
            if cur_steps <= extra_steps:
                move.append(cur_steps)
                extra_steps -= cur_steps
            else:
                move.append(extra_steps)
                extra_steps = 0
        ramp.reverse()
        move += ramp
        if steps < 0:
            for i in range(len(move)):
                move[i] = -move[i]
        return move

    def ramp2(self, ms, m1_steps, m2_steps):
        if ms > 0:
            self.sm2_cmd(ms, m1_steps, m2_steps)
            return
        msdt = self.params['accel_deltat']
        m1_ramp = self.make_ramp(m1_steps)
        m2_ramp = self.make_ramp(m2_steps)
        l1 = len(m1_ramp)
        l2 = len(m2_ramp)
        if (l1 > l2):
            m2_ramp += [0 for i in range(l1 - l2)]
        elif (l2 > l1):
            m1_ramp += [0 for i in range(l2 - l1)]
        for i in range(max(l1, l2)):
            self.sm2_cmd(msdt, m1_ramp[i], m2_ramp[i])
        
    def ramp1(self, mx, steps):
        msdt = self.params['accel_deltat']
        ramp = self.make_ramp(steps)
        for i in range(len(ramp)):
            self.sm_cmd(msdt, mx, ramp[i])
        
    def m1_move_in(self, steps):
        if self.m1_cur_pos - steps < self.params['m1_min_pos']:
            steps = self.m1_cur_pos - self.params['m1_min_pos']
        self.ramp1(1, -steps)
        self.m1_cur_pos -= steps

    def m1_move_out(self, steps):
        if self.m1_cur_pos - steps > self.params['m1_max_pos']:
            steps = self.params['m1_max_pos'] - self.m1_cur_pos
        self.ramp1(1, steps)
        self.m1_cur_pos += steps

    def m2_move_in(self, steps):
        if self.m2_cur_pos - steps < self.params['m2_min_pos']:
            steps = self.m2_cur_pos - self.params['m2_min_pos']
        self.ramp1(2, -steps)
        self.m2_cur_pos -= steps

    def m2_move_out(self, steps):
        if self.m2_cur_pos - steps > self.params['m2_max_pos']:
            steps = self.params['m2_max_pos'] - self.m2_cur_pos
        self.ramp1(2, steps)
        self.m2_cur_pos += steps

    def m1_move_abs(self, val):
        if val > self.params['m1_max_pos']:
            val = self.params['m1_max_pos']
        if val < self.params['m1_min_pos']:
            val = self.params['m1_min_pos']
        self.ramp1(1, val - self.m1_cur_pos)
        self.m1_cur_pos = val

    def m2_move_abs(self, val):
        if val > self.params['m2_max_pos']:
            val = self.params['m2_max_pos']
        if val < self.params['m2_min_pos']:
            val = self.params['m2_min_pos']
        self.ramp1(2, val - self.m2_cur_pos)
        self.m2_cur_pos = val

    def toggle_laser(self):
        # Seems to be inverted from the docs.
        if self.laser_on:
            self.sp_cmd(1);
            self.laser_on = False;
        else:
            self.sp_cmd(0);
            self.laser_on = True;

    def set_laser(self, value):
        if value:
            self.sp_cmd(0);
            self.laser_on = True;
        else:
            self.sp_cmd(1);
            self.laser_on = False;

    def run_servo(self):
        if self.cur_servo_min != self.params['servo_min']:
            self.sc_cmd(4, self.params['servo_min'])
            self.cur_servo_min = self.params['servo_min']
        if self.cur_servo_max != self.params['servo_max']:
            self.sc_cmd(5, self.params['servo_max'])
            self.cur_servo_max = self.params['servo_max']
        self.sp_cmd(0)
        time.sleep(float(self.params['servo_pause'])/1000.0)
        self.sp_cmd(1)

    def pulse_solenoid(self, ms, mx, steps):
        half_step_mode = 4
        self.send_ok('EM,{0},{0}\r'.format(half_step_mode, half_step_mode))
        self.send_ok('SM,{0},{1},{2}\r'.format(250, 0, 4))
        m1_mode = self.params['m1_mode']

    def pulse_solenoid(self):
        msdt = self.params['accel_deltat']
        self.sm_cmd(self.params['solenoid_rate'], 2, 8)

    def fix_solenoid(self):
        msdt = self.params['accel_deltat']
        self.sm_cmd(self.params['solenoid_rate'], 2, 1)

    def m1_get_cur_pos(self):
        return self.m1_cur_pos

    def m2_get_cur_pos(self):
        return self.m2_cur_pos

    def set_param(self, field, val):
        self.params[field] = val

    def get_params(self):
        return self.params

    def poll(self):
        pass

class config_gui:
    def __init__(self, motor, verbose=0):
        self.motor = motor
        self.verbose = verbose
        self.forder = [
                       'm1_min_pos',
                       'm1_max_pos',
                       'm1_mode',
                       'steps_per_sec',
                       'acceleration',
                       'accel_deltat',
                       'servo_min',
                       'servo_max',
                       'servo_pause',
                       'solenoid_rate',
                      ]
        self.descr = { 
                       'm1_min_pos'   : ('M1 Min Pos', True), 
                       'm1_max_pos'   : ('M1 Max Pos', True),
                       'm1_mode'      : ('M1 Microstep Mode (1-5)', True),
                       'steps_per_sec': ('Steps/Sec', True),
                       'acceleration' : ('Steps/Sec*2', True),
                       'accel_deltat' : ('Accel time interval (ms)', True),
                       'servo_min'    : ('Servo min setting (1-65K)', True),
                       'servo_max'    : ('Servo max setting (1-65K)', True),
                       'servo_pause'  : ('Servo pause time (ms)', True),
                       'solenoid_rate': ('Time to take 4 steps (ms)', True),
                     }
        params = self.motor.get_params()
        self.fields = dict()
        self.croot = Tk()
        self.croot.title("Config")
        self.f1 = Frame(self.croot)
        self.f1.pack(padx=15, pady=15)
        cur_row = 0
        for field in self.forder:
            (txt, can_update) = self.descr[field]
            desc = Label(self.f1, justify=LEFT, anchor=W, text=txt)
            desc.grid(row=cur_row, column=0, sticky=W)
            entry = Entry(self.f1, width=10, justify=RIGHT)
            entry.grid(row=cur_row, column=1, sticky=E)
            entry.delete(0, END)
            entry.insert(0, str(params[field]))
            if can_update:
                button = Button(self.f1, text="Update")
                button.bind("<Button-1>", lambda event, field=field : self.update_down(event, field))
                button.grid(row=cur_row, column=2, padx=5, pady=5, sticky=W) 
            else:
                button = None
            self.fields[field] = (desc, entry, button)
            cur_row += 1

    def update_down(self, event, field):
        (desc, entry, button) = self.fields[field]
        val = int(entry.get())
        if (self.verbose > 0):
            print "update_down setting", field, "to", val
        self.motor.set_param(field, val)
        params = self.motor.get_params()
        for field in self.fields:
            (desc, entry, button) = self.fields[field]
            entry.delete(0, END)
            entry.insert(0, str(params[field]))

class main_gui:
    def __init__(self, motor, verbose=0):
        self.motor = motor
        self.verbose = verbose
        self.m1_cabs_label = StringVar()

        cur_row = 0
        px = 5
        py = 5

        root.title("Doodle")
        self.f1 = Frame(root)
        self.f1.pack(padx=15, pady=15)

        self.m1_cabs = Label(self.f1, width=10, justify=RIGHT, anchor=E, textvariable=self.m1_cabs_label)
        self.m1_cabs.grid(row=cur_row, column=0, columnspan=2, sticky=W)
        self.m1_cabs_label.set("0")

        self.cabst = Label(self.f1, justify=RIGHT, anchor=E, text="Current Position")
        self.cabst.grid(row=cur_row, column=1, columnspan=2, sticky=W)

        cur_row += 1

        self.m1_abe = Entry(self.f1, width=10, justify=RIGHT)
        self.m1_abe.grid(row=cur_row, column=0, columnspan=1, sticky=W)
        self.m1_abe.delete(0, END)
        self.m1_abe.insert(0, "0")

        self.m1_b_abs = Button(self.f1, text="Goto Abs")
        self.m1_b_abs.bind("<Button-1>", lambda event: self.m1_abs_down(event))
        self.m1_b_abs.grid(row=cur_row, column=1, columnspan=2, pady=5, sticky=W)

        cur_row += 1

        self.m1_ioe = Entry(self.f1, width=10, justify=RIGHT)
        self.m1_ioe.grid(row=cur_row, column=0, columnspan=1, sticky=W)
        self.m1_ioe.delete(0, END)
        self.m1_ioe.insert(0, "1")

        self.m1_b_in = Button(self.f1, text="-")
        self.m1_b_in.bind("<Button-1>", lambda event: self.m1_in_down(event))
        self.m1_b_in.bind("<ButtonRelease-1>", lambda event: self.in_up(event))
        self.m1_b_in.grid(row=cur_row, column=1, columnspan=1, pady=5, sticky=W)

        self.m1_b_out = Button(self.f1, text="+")
        self.m1_b_out.bind("<Button-1>", lambda event: self.m1_out_down(event))
        self.m1_b_out.bind("<ButtonRelease-1>", lambda event: self.out_up(event))
        self.m1_b_out.grid(row=cur_row, column=2, columnspan=1, pady=5, sticky=W)

        cur_row += 1

        port_count=0
        devices = glob.glob("/dev/ttyUSB*")
        for device in devices:
            b = Radiobutton(self.f1, text=device, variable=port, value=device)
            b.grid(row=cur_row, column=0, columnspan=2, sticky=W)
            if port_count == 0:
                port.set(device)
            port_count += 1
            cur_row += 1 

        if port_count == 1:
            motor.set_port(device)
        elif port_count == 0:
            print "no tty ports found!"
            print "check for /dev/ttyUSB*"
            exit(1)


        cur_row += 1

        self.b_toggle_laser = Button(self.f1, text="Servo")
        self.b_toggle_laser.bind("<Button-1>", lambda event: self.servo(event))
        self.b_toggle_laser.grid(row=cur_row, column=0, columnspan=1, pady=5, sticky=W)

        cur_row += 1

        self.b_toggle_laser = Button(self.f1, text="Solenoid")
        self.b_toggle_laser.bind("<Button-1>", lambda event: self.solenoid(event))
        self.b_toggle_laser.grid(row=cur_row, column=0, columnspan=1, pady=5, sticky=W)

        cur_row += 1

        self.b_motors_off = Button(self.f1, text="Motors Off")
        self.b_motors_off.bind("<Button-1>", lambda event: self.motors_off_down(event))
        self.b_motors_off.grid(row=cur_row, column=0, columnspan=1, pady=5, sticky=W)

        cur_row += 1

        self.b_config = Button(self.f1, text="Config")
        self.b_config.bind("<Button-1>", lambda event: self.config_down(event))
        self.b_config.grid(row=cur_row, column=0, columnspan=1, pady=5, sticky=W)


    def m1_in_down(self, event):
        steps = int(self.m1_ioe.get())
        self.motor.m1_move_in(steps)

    def m2_in_down(self, event):
        steps = int(self.m2_ioe.get())
        self.motor.m2_move_in(steps)

    def in_up(self, event):
        if self.verbose > 0:
            print "in_up"

    def m1_out_down(self, event):
        steps = int(self.m1_ioe.get())
        self.motor.m1_move_out(steps)

    def m2_out_down(self, event):
        steps = int(self.m2_ioe.get())
        self.motor.m2_move_out(steps)

    def out_up(self, event):
        if self.verbose > 0:
            print "out_up"

    def m1_abs_down(self, event):
        self.motor.m1_move_abs(int(self.m1_abe.get()))

    def m2_abs_down(self, event):
        self.motor.m2_move_abs(int(self.m2_abe.get()))

    def clear_abs_down(self, event):
        self.motor.reset_counts()

    def config_down(self, event):
        self.cg = config_gui(self.motor, verbose=self.verbose)

    def servo(self, event):
        self.motor.run_servo()

    def solenoid(self, event):
        self.motor.pulse_solenoid()

    def motors_off_down(self, event):
        self.motor.motors_off()

    def poll(self):
        m1_cur_pos = self.motor.m1_get_cur_pos()
        if m1_cur_pos != int(self.m1_cabs_label.get()):
            self.m1_cabs_label.set(str(m1_cur_pos))
        m2_cur_pos = self.motor.m2_get_cur_pos()

def call_polls():
    global motor
    global gui
    motor.poll();
    gui.poll()
    root.after(100, call_polls);

if __name__ == '__main__':
    root = Tk()

    # A couple of globals
    port = StringVar()

    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='verbose', type=int,
                        help='set verbose mode')
    args = parser.parse_args()

    motor = motor(verbose=args.verbose)
    gui = main_gui(motor, verbose=args.verbose)
    root.after(20, call_polls);
    root.mainloop()
