#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs  # Import necessary for transformations

def callback(data, args):
    tf_buffer, goal_poses = args
    if len(goal_poses) >= 10:
        rospy.loginfo("Maximum number of goal poses (10) already saved.")
        rospy.signal_shutdown("Reached limit of 10 goal poses.")
        return  # Exit the callback if 10 poses are already saved

    for marker in data.markers:
        rospy.loginfo("Marker ID: %s", marker.id)
        rospy.loginfo("Position: (%s, %s, %s)", marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z)
        rospy.loginfo("Orientation: (%s, %s, %s, %s)", marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)

        try:
            transform = tf_buffer.lookup_transform('map', marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=marker.header, pose=marker.pose.pose), transform)
            rospy.loginfo("Transformed Pose in Map Frame: %s", transformed_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF2 error: %s", e)
            continue

        if input("Save this pose as goal pose? (y/n): ").strip().lower() == 'y':
            goal_poses.append(transformed_pose.pose)
            rospy.loginfo("Goal pose for marker %s saved in the map frame.", marker.id)
            if len(goal_poses) == 10:
                rospy.loginfo("Collected 10 goal poses. Stopping further input.")
                rospy.signal_shutdown("Completed collecting 10 goal poses.")
                break

def listener():
    rospy.init_node('ar_track_alvar_subscriber', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    goal_poses = []  # List to hold up to 10 goal poses
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback, (tf_buffer, goal_poses))
    rospy.spin()

if __name__ == '__main__':
    listener()
