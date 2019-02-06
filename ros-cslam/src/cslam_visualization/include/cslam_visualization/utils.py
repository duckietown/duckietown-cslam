#!/usr/bin/env python  
import rospy
import math
import tf
from visualization_msgs.msg import Marker, MarkerArray
import rospkg
import os
import yaml

def get_trafficsign_marker(marker_id, x, y, q, marker_type):
    
    if marker_type == "T-intersection":
        marker_type = "T-intersect"

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = "traffic_signs"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://duckietown_visualization/meshes/traffic-signs/" + \
                           "sign_" + marker_type.replace('-','_') + ".dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = -0.04

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    (_,_,yaw) = tf.transformations.euler_from_quaternion(q)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw-math.pi/2)

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker


def get_apriltag_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = "ground_tags"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://duckietown_visualization/meshes/apriltags/apriltag.dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = -0.04

    marker.scale.x = 0.064
    marker.scale.y = 0.064
    marker.scale.z = 1
    
    (_,_,yaw) = tf.transformations.euler_from_quaternion(q)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
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
    marker.mesh_resource = "package://duckietown_visualization/meshes/watchtower/watchtower.dae"
    marker.mesh_use_embedded_materials = True
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    
    (_,_,yaw) = tf.transformations.euler_from_quaternion(q)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

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
    
    (_,_,yaw) = tf.transformations.euler_from_quaternion(q)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker

def get_markers(duckiebots,watchtowers,apriltags,listener):

    trafficsign_apriltags = get_trafficsign_apriltags()
    
    marker_array = MarkerArray()

    for it in range(len(duckiebots)): 
        try:
            (trans,rot) = listener.lookupTransform('/map', \
                duckiebots[it], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            continue
        tag_id = int(duckiebots[it].split('_')[-1])
        marker_array.markers.append(get_duckiebot_marker(tag_id,trans[0],trans[1],rot))
    
    for it in range(len(watchtowers)): 
        try:
            (trans,rot) = listener.lookupTransform('/map', \
                watchtowers[it], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            continue
        tag_id = int(watchtowers[it].split('_')[-1])
        marker_array.markers.append(get_watchtower_marker(tag_id,trans[0],trans[1],rot))
    
    for it in range(len(apriltags)): 
        try:
            (trans,rot) = listener.lookupTransform('/map', \
                apriltags[it], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            continue
        tag_id = int(apriltags[it].split('_')[-1])

        if tag_id not in trafficsign_apriltags:
            marker_array.markers.append(get_apriltag_marker(tag_id,trans[0],trans[1],rot))
        else:
            marker_type = trafficsign_apriltags[tag_id]
            marker_array.markers.append(get_trafficsign_marker(tag_id,trans[0],trans[1],rot,marker_type))

    return marker_array

def get_trafficsign_apriltags():
    rospack = rospkg.RosPack()
    graph_builder_path = rospack.get_path('pose_graph_builder')

    apriltags_DB = os.path.join(graph_builder_path, 'data', 'apriltagsDB.yaml')
    
    tag_map = {}
    
    # Read YAML file.
    with open(apriltags_DB, 'r') as stream:
        try:
            complete_dict = yaml.safe_load(stream)
            tag_map = {tag['tag_id']:tag['traffic_sign_type'] \
                for tag in complete_dict \
                if tag['tag_type'] == 'TrafficSign'}
        except yaml.YAMLError as exc:
            print(exc)
    return tag_map
