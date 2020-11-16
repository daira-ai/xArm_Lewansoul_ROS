#!/usr/bin/env python

import rospy
import time
from control_msgs.msg import GripperCommandActionGoal

def move_gripper(ratio=0.0):
    gripper_command = GripperCommandActionGoal()
    gripper_command.header.seq = 0
    gripper_command.goal.command.position=ratio
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

def open_gripper():
    gripper_command = GripperCommandActionGoal()
    gripper_command.header.seq = 0
    gripper_command.goal.command.position=1.0
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

def close_gripper():
    gripper_command = GripperCommandActionGoal()
    gripper_command.header.seq = 0
    gripper_command.goal.command.position = 0.0
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

if __name__ == '__main__':
    try:
        rospy.init_node('xarm_commander', anonymous=True)

        pub = rospy.Publisher('/xarm/controller/gripper/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)
        time.sleep(1)
        open_gripper()
        time.sleep(3)
        close_gripper()
        time.sleep(3)
        move_gripper(0.5)
        time.sleep(3)
    except rospy.ROSInterruptException:
        print "ROSInterruptException"
        pass