#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import actionlib
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import  Quaternion,TransformStamped, PoseStamped, Pose
from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult


class RobotMoverActionServer():

    _result = WeldingPathResult()
    _feedback = WeldingPathFeedback()

    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # def transform_poses (self, poses_in):
        
    #     # Create a TransformStamped message
    #     transform_stamped = TransformStamped()
    #     transform_stamped.transform.translation.x = 0.016
    #     transform_stamped.transform.translation.y = 0
    #     transform_stamped.transform.translation.z = 0
    #     transform_stamped.transform.rotation.x = -0.098
    #     transform_stamped.transform.rotation.y = -0.9636305
    #     transform_stamped.transform.rotation.z = 0
    #     transform_stamped.transform.rotation.w = 0.2672384
    #     transform_stamped.header.frame_id = "tool0"  
    #     transform_stamped.child_frame_id = "nozzle_point"

    #     # Initialize tf2 buffer and listener
    #     tf_buffer = tf2_ros.Buffer()
    #     tf_listener = tf2_ros.TransformListener(tf_buffer)

    #     # Initialize transformed poses list
    #     poses_out_transformed = []

    #     try:
    #         nozzle_tip_to_base_link = tf_buffer.Aachen2024
    ("base_link", "nozzle_point", rospy.Time(0), rospy.Duration(1.0))
    #     except tf2_ros.TransformException as ex:
    #         rospy.logerr("Transform exception: %s", ex)
    #         return None

    #     # Lookup the transformation between the TCP frame and the nozzle tip frame
    #     try:
    #         tcp_to_nozzle_tip = tf_buffer.lookup_transform("nozzle_point", "tool0", rospy.Time(0), rospy.Duration(1.0))
    #     except tf2_ros.TransformException as ex:
    #         rospy.logerr("Transform exception: %s", ex)
    #         return None

    #     # Combine transformations to compute transformation between TCP frame and base_link frame
    #     try:
    #         tcp_to_base_link = tf2_geometry_msgs.do_transform_pose(tcp_to_nozzle_tip, nozzle_tip_to_base_link)
    #     except tf2_ros.TransformException as ex:
    #         rospy.logerr("Transform exception: %s", ex)
    #         return None

    #     # Transform each input pose
    #     for pose_in in poses_in:

    #         # Transform the input pose
    #         pose_out_stamped = tf2_geometry_msgs.do_transform_pose(pose_in, transform)

    #         # Append the transformed pose to the output list
    #         poses_out_transformed.append(pose_out_stamped.pose)
    #     return poses_out_transformed

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

        # self.move_group.setJointValueTarget([2.33874,-1.22173,1.72788,-3.4383,-2.0944,-0.488692])
        # plan=self.move_group.plan()

        # if plan:
        #     self.move_group.execute(plan,wait=True)

         
        #poses = self.transform_poses(goal.poses)
        poses=[]
        
        for ps in goal.poses:
              pos=ps.pose
              pos.position.z=0.2
              pos.orientation= Quaternion(0, -0.9636305, 0, 0.2672384 )
              poses.append(pos)


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
            self.action_server.set_succeeded(self._result)
            rospy.loginfo("Welding path execution completed.")    
        else:
            rospy.loginfo("Planning Unsuccessfull. A few or all given points are not reachable from current position. Planning Success Rate: "+str(success))
            self._result.result_code = 0
            self.action_server.set_aborted(self._result)        



if __name__ == "__main__":

    # rospy.init_node("node_name")
    moveit_commander.roscpp_initialize(sys.argv)


    server = RobotMoverActionServer()

    rospy.spin()



