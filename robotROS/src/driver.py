#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from robotROS.msg import drive

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing drive msg to drive controller!
---------------------------
Moving around:
        w    
   a         d
        s    

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0),
        'a':(0,0,0,1),
        's':(-1,0,0,0),
        'd':(0,0,0,-1),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/drive', drive, queue_size = 2)
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self,speed, turn):
    	self.condition.acquire()
    	self.speed = speed
    	self.turn = turn
    	self.condition.notify()
    	self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0)
        self.join()

    def run(self):
        tempDrive = drive()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into drive message.
            tempDrive.forward = self.speed
            tempDrive.turn = self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(tempDrive)

        # Publish stop message when thread exits.
       
        self.publisher.publish(tempDrive)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], .1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.55)
    turn = rospy.get_param("~turn", .3)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    
    print
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    speed = 0
    turn = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(speed, turn)
        print(msg)
        while(1):	
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                speed = moveBindings[key][0]
                turn = moveBindings[key][3]
            elif(key == '\x03'):
                break
            else:
                speed = 0
                turn = 0
 
            pub_thread.update(speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
