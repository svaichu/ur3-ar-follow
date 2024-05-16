#!/usr/bin/env python3

from __future__ import print_function

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from welding_robot_msgs.msg import WeldingPathAction
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathResult

def feedback_callback(feedback):
    rospy.loginfo(f"Received feedback: {feedback.feedback_code}")

def main():
    rospy.init_node("welding_path_client")

    # Create an action client
    client = actionlib.SimpleActionClient("welding_path_action", WeldingPathAction)
    
    client.wait_for_server()

    #create pose1, pose2, pose3
    pose1 = PoseStamped()
    pose2 = PoseStamped()
    pose3 = PoseStamped()
    pose4 = PoseStamped()
    pose5 = PoseStamped()
    pose6 = PoseStamped()
    pose7 = PoseStamped()

    pose1.header.frame_id = "base_link" 
    pose1.pose.position = Point(-0.535, 0.135, 0.1)  
    pose1.pose.orientation = Quaternion(1, 0.0, 0.0, 0)
    
    

    pose2.header.frame_id = "base_link" 
    pose2.pose.position = Point(-0.37, 0.135, 0.1)  
    pose2.pose.orientation = Quaternion(1, 0.0, 0.0, 0)  
    

    pose3.header.frame_id = "base_link" 
    pose3.pose.position = Point(-0.34, 0.11, 0.1)  
    pose3.pose.orientation = Quaternion(1, 0.0, 0.0, 0)  
    
    pose4.header.frame_id = "base_link" 
    pose4.pose.position = Point(-0.34, -0.055, 0.1)  
    pose4.pose.orientation = Quaternion(1, 0.0, 0.0, 0) 
    
    pose5.header.frame_id = "base_link" 
    pose5.pose.position = Point(-0.46, 0.03, 0.1)  
    pose5.pose.orientation = Quaternion(1, 0.0, 0.0, 0) 
    
    pose6.header.frame_id = "base_link" 
    pose6.pose.position = Point(-0.46, -0.1, 0.1)  
    pose6.pose.orientation = Quaternion(1, 0.0, 0.0, 0)
    
    pose7.header.frame_id = "base_link" 
    pose7.pose.position = Point(-0.38, 0.09, 0.1)  
    pose7.pose.orientation = Quaternion(1, 0.0, 0.0, 0)
    
    

    goal=WeldingPathGoal()
    goal.poses = [pose1, pose2, pose3, pose4, pose5, pose6, pose7 ]

    # Send the goal
    client.send_goal(goal, feedback_cb=feedback_callback)

    # Wait for the result
    client.wait_for_result()
    
    #print out the result
    return client.get_result()

    if result.result_code == 1:
        rospy.loginfo("Welding path execution successful.")
    else:
        rospy.logerr("Welding path execution failed.")

if __name__ == "__main__":
    main()



