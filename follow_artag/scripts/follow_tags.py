#!/usr/bin/env python3

import rospy
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_conversions
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
import actionlib
import math

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathActionFeedback
from welding_robot_msgs.msg import WeldingPathActionGoal
from welding_robot_msgs.msg import WeldingPathActionResult
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult



def ar_tag_estimate_callback(msg):
    """Callback from topic subscriber."""
    markers = msg.markers
    #send goal to action server
    goal = WeldingPathGoal()
    goal.header = markers.header
    





def main():
    ...



if __name__ == "__main__":
    rospy.init_node("a_node_name")

    rospy.Subscriber("/a_topic_name", AlvarMarkers, ar_tag_estimate_callback)
    client = actionlib.SimpleActionClient("name-of-action-server", WeldingPathAction)


    main()

