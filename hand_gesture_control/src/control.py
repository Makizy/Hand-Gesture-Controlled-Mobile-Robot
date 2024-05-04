#!/usr/bin/env python


import rospy
import sys
import getch
from geometry_msgs.msg import Twist
from std_msgs.msg import String


directionInfo = """
Taking input from vision mediapipe and publishing to /cmd_vel


Movement:
  w             up  
a   d       left right
  s            down




Hit esc to exit
-----------------
"""


current_input = None  # Global variable to store the current gesture command


def callback(gesture):
    global current_input
    current_input = gesture.data  # Update the current command
    rospy.loginfo(f"Gesture received: {current_input}")


def velocityOut(speed, turn):  # for printing/getting speed & turn angle
    return "speed & turn angle: \tspeed %s\tturn %s " % (speed, turn)


def control():
    global pub, speed, turn, x, y, z, th, current_input
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # publishing to cmd_vel
    sub = rospy.Subscriber('gesture_topic', String, callback)  # subscribing to gesture_topic
    rospy.init_node('control')


    speed = rospy.get_param("~speed", 0.5)  # default speed val
    turn = rospy.get_param("~turn", 1.0)  # default turn val
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0


    while not rospy.is_shutdown():
        if current_input:
            if current_input == 'W':
                x = 1
            elif current_input == 'S':
                x = -1
            elif current_input == 'A':
                th = 1
            elif current_input == 'D':
                th = -1
            elif current_input == 'P':  # No movement detected
                x = 0
                y = 0
                z = 0
                th = 0
            rospy.loginfo(velocityOut(speed, th*turn))


            # Publish the velocities
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)


            # Reset the input to prevent continuous execution
            current_input = None


        rospy.sleep(0.1)  # Sleep to give time for callbacks to process


if __name__ == '__main__':
    try:
        print(directionInfo)
        control()
    except rospy.ROSInterruptException:
        pass





