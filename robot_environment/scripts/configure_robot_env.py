#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Pose, Quaternion
import sys

import rospkg


import moveit_commander

def shutdown():
    """When node is shut down, remove the previous added objects from the planning scene."""
    scene.remove_world_object()
    scene.remove_attached_object("tool0", "welding_gun")


def initialize_planning_obstacles():
    """Fill this method for adding obstacles"""

    # Configure pose of obstacle
    table_pose = PoseStamped()
    
    table_pose.header.frame_id = "base_link"  # Assuming the table's frame is in the robot's base frame
    table_pose.pose.position = Point(0.0, 0.0, -0.01)  # Position of the obstacle in meters (x, y, z)
    table_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Orientation (quaternion) of the obstacle
    
    table_normal = (0.0, 0.0, 1)  
    
    # add it to the planning scene
    #scene.add_XYZ()
    scene.add_plane("table", table_pose, normal=table_normal)

    # Configure pose of Camera Vertical
    camera_vertical_pose = PoseStamped()
    
    camera_vertical_pose.header.frame_id = "base_link"  # Assuming the table's frame is in the robot's base frame
    camera_vertical_pose.pose.position = Point(-0.61, 0.0, 0.0)  # Position of the obstacle in meters (x, y, z)
    camera_vertical_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Orientation (quaternion) of the obstacle
    
    camera_vertical_normal = (-1, 0.0, 0)  # Dimensions of the box in meters (x, y, z)

    # add it to the planning scene
    #scene.add_XYZ()
    scene.add_plane("camera_vertical", camera_vertical_pose, normal=camera_vertical_normal)

    # Configure pose of Camera Horizontal

    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"  # Assuming the obstacle's frame is in the robot's base frame
    box_pose.pose.position = Point(-0.5025, 0.0, 0.77)  # Position of the obstacle in meters (x, y, z)
    box_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Orientation (quaternion) of the obstacle
    
    box_size = (0.1075, 1, 0.2)  # Dimensions of the box in meters (x, y, z)
    
    # Add the box obstacle to the planning scene
    scene.add_box("box_obstacle", box_pose, size=box_size)





def initialize_nozzle():
    """Fill this method for adding the nozzle to the planning scene.
    It's important, otherwise the nozzle can collide with the robot/ the environment."""

    # path to nozzle
    rospack = rospkg.RosPack()
    nozzle_path = rospack.get_path("robot_environment") + "/meshes/" + "welding_nozzle.stl"

    # pose of nozzle relative to frame tool0
    nozzle_pose = PoseStamped()
    nozzle_pose.header.frame_id = "tool0"
    nozzle_pose.pose.orientation = Quaternion(0.7071068, 0, 0, 0.7071068)
    nozzle_pose.pose.position = Point(0.0, 0.0, 0.0001)
    
    

    #scene.attach_XYZ()
    scene.attach_mesh("tool0", "welding_gun", nozzle_pose, nozzle_path, size=[0.01,0.01,0.01])



if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()

    rospy.init_node("configure_robot_env_node")
    
    initialize_planning_obstacles()
    initialize_nozzle()

    rospy.on_shutdown(shutdown)
    
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)





    
    
