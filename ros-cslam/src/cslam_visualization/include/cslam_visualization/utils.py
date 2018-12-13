#!/usr/bin/env python  
import rospy
import math
import tf
from visualization_msgs.msg import Marker, MarkerArray


def get_apriltag_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = "ground_tags" #TODO

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://duckietown_visualization/meshes/apriltags/apriltag.dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 0.064
    marker.scale.y = 0.064
    marker.scale.z = 1

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker

def get_watchtower_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = "watchtowers"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://cslam_visualization/meshes/watchtower/watchtower.dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    return marker

def get_duckiebot_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = "duckiebots"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot.dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker

def get_markers(duckiebots,watchtowers,apriltags,listener):
    marker_array = MarkerArray()

    for it in range(len(duckiebots)): 
        try:
            (trans,rot) = listener.lookupTransform('/map', \
                duckiebots[it], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            continue
        marker_array.markers.append(get_duckiebot_marker(it,trans[0],trans[1],rot))
    
    for it in range(len(watchtowers)): 
        try:
            (trans,rot) = listener.lookupTransform('/map', \
                watchtowers[it], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            continue
        marker_array.markers.append(get_watchtower_marker(it,trans[0],trans[1],rot))
    
    for it in range(len(apriltags)): 
        try:
            (trans,rot) = listener.lookupTransform('/map', \
                apriltags[it], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            continue
        marker_array.markers.append(get_apriltag_marker(it,trans[0],trans[1],rot))
    
    return marker_array
