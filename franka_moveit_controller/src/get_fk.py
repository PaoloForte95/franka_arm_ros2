#! /usr/bin/env/python


import sys
import copy
import rospy

import moveit_commander
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_fk', anonymous=True)

arm = moveit_commander.MoveGroupCommander('arm')
rospy.wait_for_service('compute_fk')

try:
    moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
except rospy.ServiceException, e:
    rospy.logerror('Service call failed: %s' %e)


rqst = GetPostionFKRequest()
rqst.header.frame_id = "world"
rqst.fk_link_names = ['link6']
rqst.robot_state.joint_state.name = []
rqst.robot_state.joint_state.postion = []

i = 1
while (i < 7):
    rqst.robot_state.joint_state.name.append('id_' + str(i))
    rqst.robot_state.joint_state.postion.append(0.8)
    i+=1

res = moveit_fk(rqst)
rospy.loginfo(['computed Fk:', res])
