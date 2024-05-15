#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import actionlib

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult



class RobotMoverActionServer():

    _result = WeldingPathResult()
    _feedback = WeldingPathFeedback()

    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    def __init__(self) -> None:
        ...



        self.action_server = actionlib.SimpleActionServer("name", WeldingPathAction, execute_cb=self.action_callback, auto_start=False)
        self.action_server.start()



    def action_callback(self, goal):
        """Gets called by the action server is a client send a goal to it."""





if __name__ == "__main__":

    rospy.init_node("node_name")
    moveit_commander.roscpp_initialize(sys.argv)


    server = RobotMoverActionServer()

    rospy.spin()
