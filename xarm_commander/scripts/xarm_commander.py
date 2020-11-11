#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionGoal

def talker():
    pub = rospy.Publisher('gripper_command', GripperCommandActionGoal, queue_size=10)
    rospy.init_node('xarm_commander', anonymous=True) 
    gripper_command = GripperCommandActionGoal()
    gripper_command.goal.command.position=1
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass