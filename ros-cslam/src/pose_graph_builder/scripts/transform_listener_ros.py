#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB
import g2o
import numpy as np
import geometry as g

import tf_conversions
import tf2_ros

mygraph = dGB.duckietownGraphBuilder()
old_stamps = {}

n = 0


def callback(data):
    global n

    a = rospy.get_time()

    # global mygraph
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    t = [data.transform.translation.x,
         data.transform.translation.y, data.transform.translation.z]
    # to transform to a rotation matrix!
    q = [data.transform.rotation.x, data.transform.rotation.y,
         data.transform.rotation.z, data.transform.rotation.w]
    M = g.rotations.rotation_from_quaternion(np.array(q))
    transform = g2o.Isometry3d(M, t)
    id0 = data.header.frame_id
    id1 = data.child_frame_id

    fixed = False
    time = Time(data.header.stamp)
    if id0 == "daddy":
        fixed = id0 = 0

    old_time_stamp = 0
    time_stamp = time.data.secs % 10000 + \
        (int(time.data.nsecs * 10**(-7)))*10**(-2)

    if(id0 == id1):
        if(id0 in old_stamps):
            old_time_stamp = old_stamps[id0]
        old_stamps[id0] = time_stamp

    mygraph.add_edge(id0, id1, transform, time_stamp, old_time_stamp)
    n += 1
    if(n == 10):
        mygraph.optimize(
            15, output_name="/home/amaury/test.g2o", save_result=False, verbose=False)
        n = 0

    pose_dict = mygraph.get_all_poses()
    for node_type, node_list in pose_dict.iteritems():
        for node_id, node_pose in node_list.iteritems():
            tfbroadcast(node_type, node_id, node_pose)
    b = rospy.get_time()
    diff = b-a
    print("difference time is %f " % diff)


def tfbroadcast(node_type, node_id, node_pose):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "%s_%s" % (node_type, node_id)
    t.transform.translation.x = node_pose.t[0]
    t.transform.translation.y = node_pose.t[1]
    t.transform.translation.z = node_pose.t[2]
    q = g.rotations.quaternion_from_rotation(node_pose.R)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("transform_stamped", TransformStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def main():
    listener()


if __name__ == '__main__':
    main()
