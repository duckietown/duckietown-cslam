#!/usr/bin/env python
import os
import random
import threading
import signal
import sys
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
from nav_msgs.msg import Odometry, Path
from duckietown_msgs.msg import AprilTagDetection


class PointBroadcaster(threading.Thread):
    def __init__(self, dictionnary):
        threading.Thread.__init__(self)
        self.pose_dict = dictionnary
        self.br = tf2_ros.TransformBroadcaster()

    def tfbroadcast(self, node_id, node_pose):
        """ Brodcasts a node in the tree of transforms with TF.

            Args:
                node_id: ID of the node.
                node_pose: Pose of the node.
        """
        # Create broadcaster and transform.
        if(node_pose == None):
            print("No pose given for %s" % node_id)
            return
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        # Set frame ID. TODO: change it depending on the node type.
        if (node_id.startswith("duckiebot")):
            t.header.frame_id = "map"
            t.child_frame_id = node_id

        else:
            t.header.frame_id = "map"
            t.child_frame_id = node_id

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
        R = node_pose.R

        # R[np.where(np.abs(R) < 0.00001)] = 0
        # q = [1, 0, 0, 0]
        try:
            q = g.rotations.quaternion_from_rotation(R)
            f = rospy.get_time()
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
            # Send the transform.
            self.br.sendTransform(t)
        except:
            print("bad rotation %s" % str(R))
        # print("Proportion sendTransform/total fonction : %f" % ((c-b)/(c-a)))
        # print("Proportion quaternion/total fonction : %f" % ((f-e)/(c-a)))

    def run(self):
        for node_id, node_pose in self.pose_dict.iteritems():
            self.tfbroadcast(node_id, node_pose)


class PathBroadcaster(threading.Thread):
    def __init__(self, dictionnary):
        threading.Thread.__init__(self)
        self.path_dict = dictionnary
        self.path = Path()
        self.publisher = rospy.Publisher(
            '/movable_path', Path, queue_size=10)
        self.colors = [[0, 0, 1], [0, 1, 0], [1, 0, 0],
                       [0, 1, 1], [1, 0, 1], [1, 1, 0], [1, 1, 1]]

    def path_broadcast(self, node_id, node_path, color_index):
        """ Brodcasts the path for the node

            Args:
                node_id: ID of the node.
                node_path: Path of the node. Dictionnary of {timestamp : g2oTransform}
        """
        # Create broadcaster and transform.
        # Set frame ID. TODO: change it depending on the node type.
        self.path.header.stamp = rospy.Time.now()
        if (node_id.startswith("duckiebot")):
            self.path.header.frame_id = "map"
        else:
            self.path.header.frame_id = "map"
        # Set frame ID.
        # line_strip.id = int(node_id.split("_")[1])
        # line_strip.color.r = self.colors[color_index][0]
        # line_strip.color.g = self.colors[color_index][1]
        # line_strip.color.b = self.colors[color_index][2]
        # line_strip.color.a = 1.0
        # line_strip.scale.x = 0.01
        # Set transform:
        # - Create translation vector.
        for time_stamp in sorted(node_path.keys()):
            node_pose = self.path_dict[node_id][time_stamp]
            pose_stamped = geometry_msgs.msg.PoseStamped()
            R = node_pose.R
            # R[np.where(np.sabs(R) < 0.00001)] = 0
            # q = [1, 0, 0, 0]

            try:
                q = g.rotations.quaternion_from_rotation(R)

                pose = Pose()
                pose.position.x = node_pose.t[0]
                pose.position.y = node_pose.t[1]
                pose.position.z = node_pose.t[2]
                pose.orientation.w = q[0]
                pose.orientation.x = q[1]
                pose.orientation.y = q[2]
                pose.orientation.z = q[3]
                pose_stamped.pose = pose
                pose_stamped.header.stamp.secs = int(time_stamp)
                pose_stamped.header.stamp.nsecs = (
                    time_stamp - int(time_stamp)) * 10**9

                # point.z = 0.001
                self.path.poses.append(pose_stamped)
            except:
                print("bad rotation %s" % str(R))
        # - Create rotation matrix.
        #   Verify that the rotation is a proper rotation.
        # det = np.linalg.det(node_pose.R)
        # if (det < 0):
        #     print("after optim : det = %f" % det)
        #   NOTE: in Pygeometry, quaternion is the (w, x, y, z) form.
        # t.transform.rotation.w = q[0]
        # t.transform.rotation.x = q[1]
        # t.transform.rotation.y = q[2]
        # t.transform.rotation.z = q[3]
        # Send the transform.
        self.publisher.publish(self.path)
        # print("Proportion sendTransform/total fonction : %f" % ((c-b)/(c-a)))
        # print("Proportion quaternion/total fonction : %f" % ((f-e)/(c-a)))

    def run(self):
        color_index = 0
        for node_id, node_path in self.path_dict.iteritems():
            self.path_broadcast(node_id, node_path, color_index)
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
        self.max_iteration = rospy.get_param("maximum_g2o_iterations")
        self.minimum_edge_number_for_optimization = rospy.get_param(
            "minimum_edge_number_for_optimization")
        self.rejection_sampling_int = rospy.get_param("rejection_sampling_int")
        self.max_number_same_edge = rospy.get_param("max_number_same_edge")
        self.verbose = rospy.get_param("optim_verbose")
        self.save_output = rospy.get_param("save_g2o_output")
        self.optimization_frequency = rospy.get_param("optimization_frequency")
        self.online_optimization = rospy.get_param("online_optimization")

        self.num_messages_received = 0
        self.edge_counters = dict()
        self.optimization_period = 1.0/self.optimization_frequency
        self.pose_graph = None
        self.old_odometry_stamps = {}
        self.id_map = {}

    def initialize_id_map(self):
        """ Loads April tags into the ID map, assigning each tag in the database
            its ID and its type (e.g. TrafficSign, Localization, etc.).
        """
        config_folder = rospy.get_param("config_folder")
        aprilstagDB = "%s/%s" % (config_folder, "apriltagsDB.yaml")
        aprilstagDB_custom = "%s/%s" % (config_folder,
                                        "apriltagsDB_custom.yaml")
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
                print("apriltagDB file not found at %s" % apriltagfile)

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

    def handle_odometry_message(self, node_id, transform, time_stamp):
        """Processes an odometry message, adding an edge to the graph and
           keeping track of the last time stamp before each new odometry message
           (needed to handle edges in the pose graph and connect nodes).

           Args:
               node_id: ID of the object sending the odometry message.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # By default assign the time stamp of the last odometry message to be at
        # time 0. This is needed when the actual first odometry message for a
        # certain ID is read.
        old_time_stamp = 0

        # Get the time stamp of the previous odometry message and update the
        # time stamp of the last odometry message with the current timestamp.
        if (node_id in self.old_odometry_stamps):
            old_time_stamp = self.old_odometry_stamps[node_id]
        self.old_odometry_stamps[node_id] = time_stamp

        # Add edge to the graph.
        return self.pose_graph.add_edge(node_id, node_id, transform, time_stamp, old_time_stamp)

    def handle_watchtower_message(self, node_id0, node_id1, transform, time_stamp):
        """Processes a message containing the pose of an object seen by a
           watchtower and adds an edge to the graph. If the object seen is a
           Duckiebot, adjusts the pose accordingly.

           Args:
               node_id0: ID of the object (watchtower) that sees the April tag of the
                    other object.
               node_id1: ID of the object whose April tag is seen by the watchtower.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # Get type of the object seen.
        type_of_object_seen = node_id1.split("_")[0]

        if (type_of_object_seen == "duckiebot"):
            print("watzchtower %s is seing duckiebot %s" %
                  (node_id0, node_id1))
            # In case of Duckiebot the pose needs to be adjusted to take into
            # account the pose of the April tag w.r.t. the base frame of the
            # Duckiebot.
            t = [0.0, 0.0, 0.055]
            z_angle = 90
            x_angle = 178
            z_angle = np.deg2rad(z_angle)
            x_angle = np.deg2rad(x_angle)
            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
            R = np.matmul(R_x, R_z)  # verified!
            H_apriltag_to_base = g2o.Isometry3d(R, t)
            transform = transform * H_apriltag_to_base.inverse()

            # Add edge to the graph.
            return self.pose_graph.add_edge(node_id0, node_id1, transform, time_stamp)
        else:
            # Add edge to the graph.
            april_tag_number = int(node_id1.split("_")[1])
            if(april_tag_number < 300 or april_tag_number > 499):
                return
            if node_id0 not in self.edge_counters:
                self.edge_counters[node_id0] = dict()
            if node_id1 not in self.edge_counters[node_id0]:
                self.edge_counters[node_id0][node_id1] = 0

            if(self.edge_counters[node_id0][node_id1] < self.max_number_same_edge):
                self.pose_graph.add_edge(
                    node_id0, node_id1, transform, time_stamp)
                self.edge_counters[node_id0][node_id1] += 1
            else:
                a = random.randint(0, self.rejection_sampling_int)
                if(a == 0):
                    self.edge_counters[node_id0][node_id1] += 1

                    return self.pose_graph.add_edge(node_id0, node_id1, transform, time_stamp)

    def handle_duckiebot_message(self, node_id0, node_id1, transform, time_stamp):
        """Processes a message containing the pose of an object seen by a
           Duckiebot and adds an edge to the graph. Note: we assume that a
           Duckiebot cannot see the April tag of another Duckiebot, so no
           adjustment based on the object seen is needed.

           Args:
               node_id0: ID of the object (Duckiebot) that sees the April tag of the
                    other object.
               node_id1: ID of the object whose April tag is seen by the Duckiebot.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        # Get type of the object that sees the other object, for a sanity check.
        type_of_object_seeing = node_id0.split("_")[0]
        if (type_of_object_seeing == "duckiebot"):
            # The pose needs to be adjusted to take into account the relative
            # pose of the camera on the Duckiebot w.r.t. to the base frame of
            # the Duckiebot.
            t = [0.075, 0.0, 0.025]
            # This angle is an estimate of the angle by which the plastic
            # support that holds the camera is tilted.
            x_angle = -102
            z_angle = -90
            x_angle = np.deg2rad(x_angle)
            z_angle = np.deg2rad(z_angle)
            R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)

            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R = np.matmul(R_z, R_x)  # verified
            H_base_to_camera = g2o.Isometry3d(R, t)
            transform = H_base_to_camera * transform
        else:
            print("This should not be here! %s " % node_id0)

        # Add edge to the graph.
        # return self.pose_graph.add_edge(node_id0, node_id1, transform, time_stamp)

    def filter_name(self, node_id):
        """ Converts the frame IDs of the objects in the ROS messages (e.g.,
            Duckiebots, watchtowers, etc.) to the format <type>_<tag_id>, where
            <type> should be one of the types defined in DuckietownGraphBuilder
            (e.g. "duckiebot", "watchtower", "apriltag") and <tag_id> is the ID of
            the April tag of the object in the ID map.

            Args:
                node_id: Frame ID in the ROS message, to be converted.

            Returns:
                Converted frame ID.
        """
        if (node_id.startswith("watchtower")):
            node_id = "watchtower_%d" % int(node_id.strip("watchtower"))

        if (node_id.startswith("demowatchtower")):
            node_id = "watchtower_%d" % int(node_id.strip("demowatchtower"))

        elif (len(node_id.split("_")) == 1):
            node_id = self.find_vertex_name(node_id)

        return node_id

    def transform_callback(self, data, msg_type):
        """ ROS callback.
        """
        self.num_messages_received += 1
        # Get frame IDs of the objects to which the ROS messages are referred.
        node_id0 = data.header.frame_id
        if msg_type == "AprilTagDetection":
            node_id1 = str(data.tag_id)
        elif msg_type == "TransformStamped":
            node_id1 = data.child_frame_id
        else:
            raise Exception(
                "Transform callback received unsupported msg_type %s" % msg_type)

        if(node_id1 == "407"):
            print("\t\t\t\t !!!!!!!!! %s should be seing donald" % node_id0)
        # Convert the frame IDs to the right format.
        node_id0 = self.filter_name(node_id0)
        node_id1 = self.filter_name(node_id1)

        # Ignore messages from one watchtower to another watchtower (i.e.,
        # odometry messages between watchtowers). TODO: check if we can avoid
        # sending these messages.
        is_from_watchtower = False
        if (node_id0.startswith("watchtower")):
            is_from_watchtower = True
            if (node_id1.startswith("watchtower")):
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

        if (node_id1 == node_id0):
            # Same ID: odometry message, e.g. the same Duckiebot sending
            # odometry information at different instances in time.
            self.handle_odometry_message(
                node_id1, transform, time_stamp)
        elif (is_from_watchtower):
            # Tag detected by a watchtower.
            self.handle_watchtower_message(
                node_id0, node_id1, transform, time_stamp)
        else:
            # Tag detected by a Duckiebot.
            self.handle_duckiebot_message(
                node_id0, node_id1, transform, time_stamp)

    def optimization_callback(self, timer_event):
        if (self.num_messages_received >= self.minimum_edge_number_for_optimization):
            # a = rospy.get_time()
            self.pose_graph.optimize(
                self.max_iteration,
                save_result=self.save_output,
                verbose=self.verbose,
                output_name="/tmp/output.g2o",
                online=self.online_optimization)
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
                                              "robotarium2.yaml")
        priors_filename = "%s/%s" % (rospy.get_param("config_folder"),
                                     "priors.yaml")
        stocking_time = rospy.get_param("stocking_time")
        using_priors = rospy.get_param("using_priors")
        result_folder = rospy.get_param("result_folder")
        # Build graph based on floor map.
        self.pose_graph = dGB.DuckietownGraphBuilder(
            initial_floor_april_tags=initial_floor_april_tags, stocking_time=stocking_time, priors_filename=priors_filename, using_priors=using_priors, result_folder=result_folder)
        # Initialize ID map.
        self.initialize_id_map()
        # Subscribe to topics.
        rospy.Subscriber("/poses_acquisition/poses", AprilTagDetection,
                         lambda msg: self.transform_callback(msg, "AprilTagDetection"))
        rospy.Subscriber("/poses_acquisition/odometry", TransformStamped,
                         lambda msg: self.transform_callback(msg, "TransformStamped"))

        # Create a regular callback to invoke optimization on a regular basis
        rospy.Timer(rospy.Duration(self.optimization_period),
                    self.optimization_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    # def on_shutdown(self):
    #     self.pose_graph.on_shutdown()

    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C! Experiment will be saved and ended')
        self.pose_graph.on_shutdown()
        rospy.signal_shutdown(
            "Shutting down after cleaning and recording data")
        print("Every thing should be down")


def main():
    rospy.init_node('listener', anonymous=True, disable_signals=True)

    tflistener = TransformListener()
    # rospy.on_shutdown(tflistener.on_shutdown)
    signal.signal(signal.SIGINT, tflistener.signal_handler)
    signal.signal(signal.SIGTERM, tflistener.signal_handler)

    tflistener.listen()
    # rospy.signal_shutdown(reason)


if __name__ == '__main__':
    main()
