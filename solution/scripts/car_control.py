#!/usr/bin/env python
from re import T
import rospy
from prius_msgs.msg import Control
import curses
import time

rospy.init_node('teleop_car_control',anonymous=True)
rospy.loginfo('INIT')
rate=rospy.Rate(20)

steer_curr = [0]
brake_curr = [0]
acc_curr = [0]
brk = [False]

import sys,tty,termios
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def acc():
    brk[0] = False
    shift_gear('FORWARD')
    pub = rospy.Publisher('/prius', Control, queue_size = 10)
    acc = Control()
    if(acc_curr[0] < 1):
        acc_curr[0] = acc_curr[0] + 0.2
    acc.throttle = acc_curr[0]
    for _ in range (10):
        pub.publish(acc)
        rate.sleep()
    brake_curr[0] = 0

def br():  
    brk[0] = True  
    shift_gear('NEUTRAL')
    pub = rospy.Publisher('/prius', Control, queue_size = 10)
    br = Control()
    while(brake_curr[0] < 1):
        brake_curr[0] = brake_curr[0] + 0.2
    br.brake= brake_curr[0]
    for _ in range (10):
        pub.publish(br)
        rate.sleep()
    acc_curr[0] = 0
    print('break')

    

def left():
    brk[0] = False
    pub = rospy.Publisher('/prius', Control, queue_size = 10)
    left = Control()
    while(steer_curr[0] > -1):
        steer_curr[0] = steer_curr[0] - 0.1
    left.steer = steer_curr[0]
    brake_curr[0] = 0
    pub.publish(left)
    

    

def right():
    brk[0] = False
    pub = rospy.Publisher('/prius', Control, queue_size = 10)
    right = Control()
    while(steer_curr[0] < 1):
        steer_curr[0] = steer_curr[0] + 0.1
    right.steer = steer_curr[0]
    pub.publish(right)
    brake_curr[0] = 0

def back():
    brk[0] = False
    shift_gear('REVERSE')
    pub = rospy.Publisher('/prius', Control, queue_size = 10)
    b = Control()
    b.throttle = 0.4
    b.shift_gears = 3
    for _ in range (10):
        pub.publish(b)
        rate.sleep()
    acc_curr[0] = 0
    print('back')

def stay():
    while (1):
        br()
    

def shift_gear(gear_mode):
    pub = rospy.Publisher('/prius', Control, queue_size = 10)
    gear = Control()
    if gear_mode == 'NEUTRAL':
        gear.shift_gears = 1
    elif gear_mode == 'FORWARD':
        gear.shift_gears = 2
    elif gear_mode == 'REVERSE':
        gear.shift_gears = 3
    else :
        gear.shift_gears = 0
    pub.publish(gear)
    #print ("gear shifted to:" + str(gear.shift_gears), gear_mode)


def get():
        inkey = _Getch()
        while(1):
                k=inkey()
                if brk[0]:
                    br()
                if k!='':break
        if k=='\x1b[A':
                acc()
                print("accelerated")
        elif k=='\x1b[B':
                br()
                print("brake")
        elif k=='\x1b[D':
                right()
                print("right turn")
        elif k=='\x1b[C':
                left()
                print("Left turn")

        elif k == 'BBB' or k == 'bbb':
            back()

        elif k == 'SSS' or k == 'sss':
            stay()
        else:
                print("not an arrow key!")
                br()
                shift_gear('NEUTRAL')

        

if __name__ == '__main__':
    
    screen = curses.initscr()
    # putting in forward gear
    time.sleep(5)
    print('Ready to use')
    rate = rospy.Rate(10)
    while(True):
        get()
        rate.sleep()