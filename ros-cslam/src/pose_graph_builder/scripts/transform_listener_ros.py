#!/usr/bin/env python
import os
import random
import threading

import yaml

import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB
import g2o
import geometry as g
import numpy as np
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import *
from std_msgs.msg import Header, String, Time
from visualization_msgs.msg import *


class PointBroadcaster(threading.Thread):
    def __init__(self, dictionnary):
        threading.Thread.__init__(self)
        self.pose_dict = dictionnary
        self.br = tf2_ros.TransformBroadcaster()

    def tfbroadcast(self, node_type, node_id, node_pose):
        """ Brodcasts a node in the tree of transforms with TF.

            Args:
                node_type: Type of the node. Can be any of the types defined in
                           the class DuckietownGraphBuilder.
                node_id: ID of the node.
                node_pose: Pose of the node.
        """
        # Create broadcaster and transform.
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        # Set frame ID. TODO: change it depending on the node type.
        if (node_type == "duckiebot"):
            t.header.frame_id = "map"
            t.child_frame_id = "%s_%s" % ("duckiebot", node_id)

        else:
            t.header.frame_id = "map"
            t.child_frame_id = "%s_%s" % (node_type, node_id)

        # Set child frame ID.
        # Set transform:
        # - Create translation vector.
        t.transform.translation.x = node_pose.t[0]
        t.transform.translation.y = node_pose.t[1]
        t.transform.translation.z = node_pose.t[2]
        # - Create rotation matrix.
        #   Verify that the rotation is a proper rotation.
        # det = np.linalg.det(node_pose.R)
        # if (det < 0):
        #     print("after optim : det = %f" % det)
        #   NOTE: in Pygeometry, quaternion is the (w, x, y, z) form.
        e = rospy.get_time()
        q = g.rotations.quaternion_from_rotation(node_pose.R)
        f = rospy.get_time()
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        # Send the transform.
        b = rospy.get_time()
        self.br.sendTransform(t)
        c = rospy.get_time()
        # print("Proportion sendTransform/total fonction : %f" % ((c-b)/(c-a)))
        # print("Proportion quaternion/total fonction : %f" % ((f-e)/(c-a)))

    def run(self):
        for node_type, node_list in self.pose_dict.iteritems():
            for node_id, node_pose in node_list.iteritems():
                self.tfbroadcast(node_type, node_id, node_pose)


class PathBroadcaster(threading.Thread):
    def __init__(self, dictionnary):
        threading.Thread.__init__(self)
        self.path_dict = dictionnary
        self.publisher = rospy.Publisher('/movable_path', Marker)
        self.colors = [[0,0,1],[0,1,0], [1,0,0], [0,1,1],[1,0,1], [1,1,0], [1,1,1]]

    def path_broadcast(self, node_type, node_id, node_path, color_index):
        """ Brodcasts the path for the node

            Args:
                node_type: Type of the node. Can be any of the types defined in
                           the class DuckietownGraphBuilder.
                node_id: ID of the node.
                node_path: Path of the node. Dictionnary of {timestamp : g2oTransform}
        """
        # Create broadcaster and transform.
        line_strip = visualization_msgs.msg.Marker()
        line_strip.header.stamp = rospy.Time.now()
        line_strip.type = 4  # line_strip
        # Set frame ID. TODO: change it depending on the node type.
        if (node_type == "duckiebot"):
            line_strip.header.frame_id = "map"
        else:
            line_strip.header.frame_id = "map"
        # Set frame ID.
        line_strip.ns = node_type
        line_strip.id = int(node_id)
        line_strip.color.r = self.colors[color_index][0]
        line_strip.color.g = self.colors[color_index][1]
        line_strip.color.b = self.colors[color_index][2]
        line_strip.color.a = 1.0
        line_strip.scale.x = 0.01
        # Set transform:
        # - Create translation vector.
        for time_stamp in sorted(node_path.keys()):
            node_pose = self.path_dict[node_type][node_id][time_stamp]
            point = geometry_msgs.msg.Point()
            point.x = node_pose.t[0]
            point.y = node_pose.t[1]
            # point.z = node_pose.t[2]
            point.z = 0.001
            line_strip.points.append(point)
        # - Create rotation matrix.
        #   Verify that the rotation is a proper rotation.
        # det = np.linalg.det(node_pose.R)
        # if (det < 0):
        #     print("after optim : det = %f" % det)
        #   NOTE: in Pygeometry, quaternion is the (w, x, y, z) form.
        # q = g.rotations.quaternion_from_rotation(node_pose.R)
        # t.transform.rotation.w = q[0]
        # t.transform.rotation.x = q[1]
        # t.transform.rotation.y = q[2]
        # t.transform.rotation.z = q[3]
        # Send the transform.
        self.publisher.publish(line_strip)
        # print("Proportion sendTransform/total fonction : %f" % ((c-b)/(c-a)))
        # print("Proportion quaternion/total fonction : %f" % ((f-e)/(c-a)))

    def run(self):
        color_index = 0
        for node_type, node_list in self.path_dict.iteritems():
            for node_id, node_path in node_list.iteritems():
                self.path_broadcast(node_type, node_id, node_path, color_index)
                color_index += 1
                if(color_index == len(self.colors)):
                    color_index = 0


class TransformListener():
    """Listens for the transforms published by the acquisition node, associates
       each object in the map to an ID, builds an internal pose graph and
       broadcasts the (optimized) tree of transforms, to be e.g. visualized with
       duckietown-visualization.

       Attributes:
           pose_graph: Pose graph.
           old_odometry_stamps: Stores, for each ID, the time stamp of the last
                                odometry message read for the object with that
                                ID.
           id_map: Contains all April tags, mapped to their ID and type based on
                   the database read.
           last_callback: Stores the time when the last callback was started.
           optim_period: Sets the time (in seconds) that should pass before a
                         new optimization step is performed.
           optim_period_counter: Stores the time that one should look at to
                                 decide whether to perform a new optimization
                                 step.
           num_messages_received: Stores the number of transform messages
                                  received. This is used to ensure that
                                  optimization is performed only if enough
                                  messages have been received (i.e. if the graph
                                  contains enough information).
           edge_counters: Store the number of edges between watchotowers and april tags.
                          This is used to prevent adding too many times the same edge to the graph
    """

    def __init__(self):
        ## TODO
        # Remove useless objects, update docstring
        ##
        self.pose_graph = None
        self.old_odometry_stamps = {}
        self.id_map = {}
        # self.last_callback = rospy.get_time()
        self.max_iteration = 10
        self.minimum_edge_number_for_optimization = 100
        # self.optim_period = 0.15
        # self.optim_period_counter = -5.0
        self.num_messages_received = 0
        self.edge_counters = dict()
        self.rejection_sampling_int = 30
        self.max_number_same_edge = 30
        # self.time_means = [0.0, 0.0, 0.0, 0.0]
        # self.mode_count = [0, 0, 0, 0]
        self.verbose = rospy.get_param("optim_verbose")
        # self.optim_vertices_period = int(rospy.get_param("optim_vertices_period"))
        self.save_output = rospy.get_param("save_g2o_output")
        self.optimization_frequency = rospy.get_param("optimization_frequency")
        self.optimization_period = 1.0/self.optimization_frequency
        # self.lock = threading.Lock()

    def initialize_id_map(self):
        """ Loads April tags into the ID map, assigning each tag in the database
            its ID and its type (e.g. TrafficSign, Localization, etc.).
        """
        config_folder = rospy.get_param("config_folder")
        aprilstagDB = "%s/%s" % (config_folder, "apriltagsDB.yaml")
        aprilstagDB_custom = "%s/%s" % (config_folder, "apriltagsDB_custom.yaml")
        # Read YAML file.
        for apriltagfile in [aprilstagDB, aprilstagDB_custom]:
            if os.path.isfile(apriltagfile):

                with open(apriltagfile, 'r') as stream:
                    try:
                        complete_dict = yaml.safe_load(stream)
                        for myobject in complete_dict:
                            tag_id = myobject["tag_id"]
                            mytype = myobject['tag_type']
                            vehicle_name = myobject['vehicle_name']
                            self.id_map[str(tag_id)] = mytype
                            if vehicle_name:
                                # print(vehicle_name)
                                self.id_map[str(vehicle_name)] = str(tag_id)

                    except yaml.YAMLError as exc:
                        print(exc)
            else:
                print("apriltagDB file not found at %s" %apriltagfile)
    def find_vertex_name(self, id):
        """ Returns the format ID of an object in the ID map based on its type.

            Args:
                id: Original ID of the object in the ID map.

            Returns:
                ID of the objected, formatted by adding "duckiebot_" or "apriltag_"
                to it, based on its type.
        """
        vertex_type = ""
        if not id.isdigit():
            # print(id)
            id = self.id_map.get(id, "0")
            # print(id)
        
            vertex_type = self.id_map.get(id, "apriltag")
            # print(vertex_type)
        else:
            vertex_type = self.id_map.get(id, "apriltag")

        if (vertex_type == "Vehicle"):
            id = "duckiebot_%s" % id
        else:
            id = "apriltag_%s" % id

        return id

    def handle_odometry_message(self, id, transform, time_stamp):
        """Processes an odometry message, adding an edge to the graph and
           keeping track of the last time stamp before each new odometry message
           (needed to handle edges in the pose graph and connect nodes).

           Args:
               id: ID of the object sending the odometry message.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # By default assign the time stamp of the last odometry message to be at
        # time 0. This is needed when the actual first odometry message for a
        # certain ID is read.
        old_time_stamp = 0

        # Get the time stamp of the previous odometry message and update the
        # time stamp of the last odometry message with the current timestamp.
        if (id in self.old_odometry_stamps):
            old_time_stamp = self.old_odometry_stamps[id]
        self.old_odometry_stamps[id] = time_stamp

        # Add edge to the graph.
        return self.pose_graph.add_edge(id, id, transform, time_stamp, old_time_stamp)

    def handle_watchtower_message(self, id0, id1, transform, time_stamp):
        """Processes a message containing the pose of an object seen by a
           watchtower and adds an edge to the graph. If the object seen is a
           Duckiebot, adjusts the pose accordingly.

           Args:
               id0: ID of the object (watchtower) that sees the April tag of the
                    other object.
               id1: ID of the object whose April tag is seen by the watchtower.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # Get type of the object seen.
        type_of_object_seen = id1.split("_")[0]
        if (type_of_object_seen == "duckiebot"):
            # In case of Duckiebot the pose needs to be adjusted to take into
            # account the pose of the April tag w.r.t. the base frame of the
            # Duckiebot.
            t = [0.0, 0.0, 0.1]
            z_angle = 90
            z_angle = np.deg2rad(z_angle)
            x_angle = np.deg2rad(180)
            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
            R = np.matmul(R_x, R_z)
            H_apriltag_to_base = g2o.Isometry3d(R, t)
            transform = transform * H_apriltag_to_base

            # Add edge to the graph.
            return self.pose_graph.add_edge(id0, id1, transform, time_stamp)
        else:
            # Add edge to the graph.
            if id0 not in self.edge_counters:
                self.edge_counters[id0] = dict()
            if id1 not in self.edge_counters[id0]:
                self.edge_counters[id0][id1] = 0

            if(self.edge_counters[id0][id1] < self.max_number_same_edge):
                self.pose_graph.add_edge(id0, id1, transform, time_stamp)
                self.edge_counters[id0][id1] += 1
            else:
                a = random.randint(0, self.rejection_sampling_int)
                if(a == 0):
                    self.edge_counters[id0][id1] += 1

                    return self.pose_graph.add_edge(id0, id1, transform, time_stamp)

    def handle_duckiebot_message(self, id0, id1, transform, time_stamp):
        """Processes a message containing the pose of an object seen by a
           Duckiebot and adds an edge to the graph. Note: we assume that a
           Duckiebot cannot see the April tag of another Duckiebot, so no
           adjustment based on the object seen is needed.

           Args:
               id0: ID of the object (Duckiebot) that sees the April tag of the
                    other object.
               id1: ID of the object whose April tag is seen by the Duckiebot.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # Get type of the object that sees the other object, for a sanity check.
        type_of_object_seeing = id0.split("_")[0]
        if (type_of_object_seeing == "duckiebot"):
            # The pose needs to be adjusted to take into account the relative
            # pose of the camera on the Duckiebot w.r.t. to the base frame of
            # the Duckiebot.
            t = [0.1, 0.0, 0.1]
            # This angle is an estimate of the angle by which the plastic
            # support that holds the camera is tilted.
            y_angle = 105
            z_angle = -90
            y_angle = np.deg2rad(y_angle)
            z_angle = np.deg2rad(z_angle)
            R_y = g.rotation_from_axis_angle(np.array([0, 1, 0]), y_angle)

            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R = np.matmul(R_y, R_z)
            H_base_to_camera = g2o.Isometry3d(R, t)
            transform = H_base_to_camera * transform
        else:
            print("This should not be here! %s " % id0)

        # Add edge to the graph.
        return self.pose_graph.add_edge(id0, id1, transform, time_stamp)

    def filter_name(self, id):
        """ Converts the frame IDs of the objects in the ROS messages (e.g.,
            Duckiebots, watchtowers, etc.) to the format <type>_<tag_id>, where
            <type> should be one of the types defined in DuckietownGraphBuilder
            (e.g. "duckiebot", "watchtower", "apriltag") and <tag_id> is the ID of
            the April tag of the object in the ID map.

            Args:
                id: Frame ID in the ROS message, to be converted.

            Returns:
                Converted frame ID.
        """
        if (id.startswith("watchtower")):
            id = "watchtower_%d" % int(id.strip("watchtower"))

        if (id.startswith("demowatchtower")):
            id = "watchtower_%d" % int(id.strip("demowatchtower"))

        elif (len(id.split("_")) == 1):
            id = self.find_vertex_name(id)

        return id

    def transform_callback(self, data):
        """ ROS callback.
        """
        ## TODO
        # remove useless code (optimization part)
        ##

        
        # Update time of last callback and time counter for the optimization.
        # start_time = rospy.get_time()
        # self.optim_period_counter += start_time - self.last_callback
        # self.last_callback = start_time
        
        self.num_messages_received += 1
        # Get frame IDs of the objects to which the ROS messages are referred.
        id0 = data.header.frame_id
        id1 = data.child_frame_id
        # Convert the frame IDs to the right format.
        id0 = self.filter_name(id0)
        id1 = self.filter_name(id1)

        # Ignore messages from one watchtower to another watchtower (i.e.,
        # odometry messages between watchtowers). TODO: check if we can avoid
        # sending these messages.
        is_from_watchtower = False
        if (id0.startswith("watchtower")):
            is_from_watchtower = True
            if (id1.startswith("watchtower")):
                # print(data)
                return 0

        # Create translation vector.
        t = [
            data.transform.translation.x, data.transform.translation.y,
            data.transform.translation.z
        ]

        # Create rotation matrix. NOTE: in Pygeometry, quaternion is
        # the (w, x, y, z) form.
        q = [
            data.transform.rotation.w, data.transform.rotation.x,
            data.transform.rotation.y, data.transform.rotation.z
        ]
        M = g.rotations.rotation_from_quaternion(np.array(q))

        # Verify that the rotation is a proper rotation.
        det = np.linalg.det(M)
        if (det < 0):
            print("det is %f" % det)

        # Obtain complete transform and use it to add vertices in the graph.
        transform = g2o.Isometry3d(M, t)
        time = Time(data.header.stamp)
        time_stamp = time.data.secs + time.data.nsecs * 10**(-9)

        if (id1 == id0):
            # Same ID: odometry message, e.g. the same Duckiebot sending
            # odometry information at different instances in time.
            self.handle_odometry_message(
                id1, transform, time_stamp)
        elif (is_from_watchtower):
            # Tag detected by a watchtower.
            self.handle_watchtower_message(
                id0, id1, transform, time_stamp)
        else:
            # Tag detected by a Duckiebot.
            self.handle_duckiebot_message(
                id0, id1, transform, time_stamp)

        # If enough time has passed since the last optimization, perform a new
        # one and reset the optimization counter.
        # if (self.optim_period_counter > self.optim_period and self.num_messages_received >= 80 and
        #         self.num_messages_received % self.optim_vertices_period == 0):
        #     a = rospy.get_time()
        #     self.pose_graph.optimize(
        #         10,
        #         save_result=self.save_output,
        #         verbose=self.verbose,
        #         output_name="/tmp/output.g2o")
        #     self.optim_period_counter = 0
        #     b = rospy.get_time()
        #     # Broadcast tree of transforms with TF.
        #     pose_dict = self.pose_graph.get_all_poses()
        #     c = rospy.get_time()
        #     point_broadcaster = PointBroadcaster(pose_dict)
        #     point_broadcaster.start()

        #     path_dict = self.pose_graph.get_movable_paths()
        #     # print(path_dict)
        #     path_broadcaster = PathBroadcaster(path_dict)
        #     path_broadcaster.start()
        #     # for node_type, node_list in pose_dict.iteritems():
        #     #     for node_id, node_pose in node_list.iteritems():
        #     #         self.tfbroadcast(node_type, node_id, node_pose)
        #     d = rospy.get_time()
        #     print("optimize : %f \t get_poses : %f \t broadcast : %f" %
        #           (b-a, c-b, d-c))
        # end_time = rospy.get_time()
        # diff_time = end_time - start_time
        # # print(diff_time)
        # self.last_callback = rospy.get_time()

    def optimization_callback(self):
        if (self.num_messages_received >= self.minimum_edge_number_for_optimization):
            # a = rospy.get_time()
            self.pose_graph.optimize(
                self.max_iteration,
                save_result=self.save_output,
                verbose=self.verbose,
                output_name="/tmp/output.g2o")
            # b = rospy.get_time()
            
            # Broadcast tree of transforms with TF.
            pose_dict = self.pose_graph.get_all_poses()
            # c = rospy.get_time()
            
            point_broadcaster = PointBroadcaster(pose_dict)
            point_broadcaster.start()

            path_dict = self.pose_graph.get_movable_paths()
            # print(path_dict)
            path_broadcaster = PathBroadcaster(path_dict)
            path_broadcaster.start()

    def listen(self):
        """Initializes the graph based on the floor map and initializes the ID
           map. Then starts listening to the poses and odometry topics published
           by the acquistion node.
        """
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        initial_floor_april_tags = "%s/%s" % (rospy.get_param("config_folder"),
                                              "robotarium1.yaml")
        # Build graph based on floor map.
        self.pose_graph = dGB.DuckietownGraphBuilder(
            initial_floor_april_tags=initial_floor_april_tags)
        # Initialize ID map.
        self.initialize_id_map()
        # Subscribe to topics.
        rospy.Subscriber("/poses_acquisition/poses", TransformStamped,
                         self.transform_callback)
        rospy.Subscriber("/poses_acquisition/odometry", TransformStamped,
                         self.transform_callback)


        rospy.Timer(rospy.Duration(self.optimization_period), self.optimization_callback)                 
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    rospy.init_node('listener', anonymous=True)

    tflistener = TransformListener()
    tflistener.listen()


if __name__ == '__main__':
    main()
