#!/usr/bin/env python

from __future__ import print_function

import roslib; 
# roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math

from geometry_msgs.msg import Twist

import sys, select, termios, tty

MAX_SPEED = 5

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        W    
    A   S   D
        X    
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease only linear speed 0.1m/s
e/c : increase/decrease only angular speed pi/8 rad/s
CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0),
        'a':(0,0,0,1),
        's':(0,0,0,0),
        'd':(0,0,0,-1),
        'x':(-1,0,0,0),
    }

speedBindings={
        'q':(0.01,0),
        'z':(-0.01,0),
        'e':(0,math.pi/8),
        'c':(0,-math.pi/8),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rospy.init_node('control_keyboard')

    speed = rospy.get_param("~speed", 0.1)
    turn = rospy.get_param("~turn", math.pi/8)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed + speedBindings[key][0], MAX_SPEED)
                turn = min(turn + speedBindings[key][1], MAX_SPEED)

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()

            twist.linear.x = x*speed
            twist.linear.y = y*speed
            twist.linear.z = z*speed
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th*turn
            
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)