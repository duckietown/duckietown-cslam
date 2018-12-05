#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB
import g2o
import numpy as np
import geometry as g
import yaml
import tf_conversions
import tf2_ros

# mygraph = dGB.duckietownGraphBuilder(
#     initial_floor_april_tags="/home/amaury/duckietown/duckietown-cslam/lib-cslam/src/duckietown_cslam/duckietownGraphBuilder/robotarium1.yaml")
# old_stamps = {}

# id_map = {}
# # aprilstagDB = "/home/amaury/duckietown/duckietown-world/src/duckietown_world/data/apriltagsDB.yaml"
# n = 0


class transform_listener():
    def __init__(self):
        self.mygraph = None
        self.old_stamps = {}
        self.id_map = {}
        self.n = 0

    def initialize_id_map(self):
        global id_map
        config_folder = rospy.get_param("config_folder")
        aprilstagDB = "%s/%s" % (config_folder, "apriltagsDB.yaml")
        with open(aprilstagDB, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                for myobject in complete_dict:
                    tag_id = myobject["tag_id"]
                    mytype = myobject['tag_type']
                    self.id_map[str(tag_id)] = mytype
                # print(id_map)
            except yaml.YAMLError as exc:
                print(exc)

    def find_vertex_name(self, id):
        if(self.id_map[id] == "Vehicle"):
            id = "duckie_%s" % id
        else:
            id = "apriltag_%s" % id
        # print(id)
        return id

    def callback(self, data):
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

        if(id0.startswith("watchtower")):
            id0 = "watchtower_%s" % id0.strip("watchtower")

        if(len(id1.split("_")) == 1):
            id1 = self.find_vertex_name(id1)

        fixed = False
        time = Time(data.header.stamp)
        if id0 == "daddy":
            fixed = id0 = 0

        old_time_stamp = 0
        time_stamp = time.data.secs + time.data.nsecs * 10**(-9)

        if(id0 == id1):
            if(id0 in self.old_stamps):
                old_time_stamp = self.old_stamps[id0]
            self.old_stamps[id0] = time_stamp

        self.mygraph.add_edge(id0, id1, transform, time_stamp, old_time_stamp)
        self.n += 1
        if(self.n == 50):
            self.mygraph.optimize(
                15,  save_result=False, verbose=False)
            self.n = 0

        if(self.n % 20 == 0):
            pose_dict = self.mygraph.get_all_poses()
            for node_type, node_list in pose_dict.iteritems():
                for node_id, node_pose in node_list.iteritems():
                    self.tfbroadcast(node_type, node_id, node_pose)
        b = rospy.get_time()
        diff = b-a
        print("difference time is %f " % diff)

    def tfbroadcast(self, node_type, node_id, node_pose):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "%s_%s" % (node_type, node_id)
        t.transform.translation.x = node_pose.t[0]
        t.transform.translation.y = node_pose.t[1]
        t.transform.translation.z = node_pose.t[2]
        # print(node_pose.R)
        q = g.rotations.quaternion_from_rotation(node_pose.R)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        br.sendTransform(t)

    def listen(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('listener', anonymous=True)
        initial_floor_april_tags = "%s/%s" % (rospy.get_param(
            "config_folder"), "robotarium1.yaml")
        self.mygraph = dGB.duckietownGraphBuilder(
            initial_floor_april_tags=initial_floor_april_tags)
        self.initialize_id_map()

        rospy.Subscriber("/poses_acquisition/poses",
                         TransformStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    tflistener = transform_listener()
    tflistener.listen()


if __name__ == '__main__':
    main()
