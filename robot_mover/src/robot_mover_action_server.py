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
 
        rospy.init_node("robot_mover_action_server")
        moveit_commander.roscpp_initialize(sys.argv)

        self._result = WeldingPathResult()
        self._feedback = WeldingPathFeedback()

        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

        self.action_server = actionlib.SimpleActionServer("welding_path_action", WeldingPathAction, execute_cb=self.action_callback, auto_start=False)
        self.action_server.start()



    def action_callback(self, goal):
        """Gets called by the action server is a client send a goal to it."""
        
        rospy.loginfo("Received a new welding path goal.")

        self._feedback.feedback_code = 1
        self.action_server.publish_feedback(self._feedback)

        
        poses = [p.pose for p in goal.poses]
        
        # Plan Cartesian trajectory
        cartesian_traj, success = self.move_group.compute_cartesian_path(
            poses,
            0.01,  # Step size
            0.0,   # Jump threshold
            avoid_collisions=True,
            # max_velocity_scaling_factor=0.1
        )
              
        
        # Set the result code after successful execution
        if success==1:
            rospy.loginfo("Planning Successfull. Motion should be possible through all given points")
            self._result.result_code = 1
            # Execute trajectory
            self.move_group.execute(cartesian_traj, wait=True)
            
            rospy.loginfo("Welding path execution completed.")    
        else:
            rospy.loginfo("Planning Unsuccessfull. A few or all given points are not reachable from current position. Planning Success Rate: "+str(success))
            self._result.result_code = 0
            
        self.action_server.set_succeeded(self._result)
                       
        
        



if __name__ == "__main__":

    # rospy.init_node("node_name")
    moveit_commander.roscpp_initialize(sys.argv)


    server = RobotMoverActionServer()

    rospy.spin()
