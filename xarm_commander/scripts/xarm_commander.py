#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionGoal

def grasp():
    gripper_command = GripperCommandActionGoal()
    gripper_command.goal.command.position=0.0
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

def open():
    gripper_command = GripperCommandActionGoal()
    gripper_command.goal.command.position=1.0
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('gripper_command', GripperCommandActionGoal, queue_size=10)
        time.sleep(2)
        rospy.init_node('xarm_commander', anonymous=True) 
        open()
        time.sleep(5)
        grasp()
        time.sleep(5)
    except rospy.ROSInterruptException:
        pass