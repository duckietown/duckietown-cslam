import numpy as np
import g2o
import duckietown_cslam.g2oGraphBuilder.g2ograph_builder as g2oBG
import geometry as g
import random
import yaml

time_step = 0.5


class DuckietownGraphBuilder():
    """TODO

       Attributes:
           graph: Pose graph (g2oGraphBuilder object).
           types: Possible types of nodes in the graph, i.e., of objects in
                  Duckietown.
           timestamp_local_indices:
                  Dictionary of dictionaries of dictionaries that keeps track of
                  the timestamps for each node in the pose graph, for each type
                  of node. In particular, a certain node in the graph represents
                  the pose of an object w.r.t to another object and is given a
                  node_id and a node_type (e.g. "duckie", whose pose could be
                  either obtained by the Duckiebot being seen by a watchtower or
                  from an odometry message sent by the Duckiebot). The pose for
                  each node can be updated at different time stamps as new
                  messages are received. Therefore, each node can be associated
                  to different timestamps, that are kept track of by assigning
                  them a 'local index'. The latter increases as new timestamps
                  for that node are received. For a certain node, each timestamp
                  is given a 'local index' that is unique w.r.t. the other
                  timestamps of that node. Contrary to the other nodes, nodes
                  that come from odometry message are only associated to a
                  single timestamp. The timestamp time_stamp (float) associated
                  to the node with node_id and type node_type is given the
                  'local index'
                  timestamp_local_indices[node_type][node_id][time_stamp].
           movable: List of the types of nodes that, for the purposes of
                    optimization, should be considered as objects that can move
                    in Duckietown.
           num_local_indices_assigned:
                    Dictionary of dictionaries.
                    num_local_indices_assigned[node_type][node_id] stores the
                    number of 'local indices' already assigned to timestamps by
                    the node of type node_type and with ID node_id.
           last_time_stamp: Dictionary of dictionaries.
                            last_time_stamp[node_type][node_id] contains the
                            timestamp furthest in time among those associated to
                            the node of type node_type and with ID node_id.
           retro_interpolate: True if retro-interpolation should be performed,
                              False otherwise. Retro-interpolation allows to
                              handle messages with a timestamp anterior to the
                              latest timestamp of the already-processed
                              messages. See retro_interpolate method for further
                              details.
    """

    def __init__(self,
                 initial_duckie_dict={},
                 initial_watchtower_dict={},
                 initial_april_dict={},
                 initial_floor_april_tags="",
                 retro_interpolate=True):
        # Initialize pose graph.
        self.graph = g2oBG.g2oGraphBuilder()
        # Define node types.
        self.types = ["duckie", "watchtower", "apriltag"]
        # Initialize first-level dictionary of timestamp_local_indices by
        # associating each node type with the respective input dictionary of
        # IDs.
        initial_dicts = [
            initial_duckie_dict, initial_watchtower_dict, initial_april_dict
        ]
        self.timestamp_local_indices = dict(zip(self.types, initial_dicts))
        # Define movable node types.
        self.movable = ["duckie"]
        # Initialize first-level dictionary of num_local_indices_assigned.
        self.num_local_indices_assigned = dict()
        # Initialize first-level dictionary of last_time_stamp.
        self.last_time_stamp = dict()
        # Set retro-interpolate mode as inputted.
        self.retro_interpolate = retro_interpolate
        # Load the initial floor April tags if not done yet.
        if (initial_floor_april_tags != ""):
            self.load_initial_floor_april_tags(initial_floor_april_tags)

    def load_initial_floor_april_tags(self, initial_floor_april_tag_file):
        """Adds the poses of the initial floor April tags by reading them from
           file.

           Args:
               initial_floor_april_tag_file: File from where to read the poses
                                             of the floor tags.
        """
        # TODO: Use duckietown-world to load the tags.
        with open(initial_floor_april_tag_file, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                vertex_pose = g2o.Isometry3d(np.eye(3), [0, 0, 0])
                self.graph.add_vertex(0, vertex_pose, fixed=True)

                for key, value in complete_dict.iteritems():
                    if key == "objects":
                        for myobject, object_value in value.iteritems():
                            if (object_value['kind'] == "floor_tag"):
                                tag_id = object_value['tag']['~TagInstance'][
                                    'tag_id']
                                position = object_value['pose'][
                                    '~SE2Transform']['p']
                                theta = 0
                                if ("theta_deg" in object_value['pose'][
                                        '~SE2Transform']):
                                    theta = object_value['pose'][
                                        '~SE2Transform']['theta_deg']
                                vertex_id = "apriltag_%d" % tag_id
                                self.add_vertex(
                                    vertex_id,
                                    theta,
                                    position,
                                    True,
                                    fixed=True)
                print(self.timestamp_local_indices)
            except yaml.YAMLError as exc:
                print(exc)

    def add_timestamp_to_node(self, node_type, node_id, time_stamp):
        """Associates a timestamp to a node, by giving it a 'local index' in the
           dictionary timestamp_local_indices and by updating the latest
           timestamp associated to that node if necessary. If in
           retro-interpolate mode, performs retro-interpolation on the newly
           added timestamp if needed.

           Args:
               node_type: Type of the node to which the timestamp should be
                          associated.
               node_ID: ID of the node to which the timestamp should be
                        associated.
               time_stamp: Timestamp in seconds.

           Returns:
               True if the operation is successful, False otherwise (i.e., if
               the timestamp is already associated to the node with a 'local
               index' or if the node is a Duckiebot for which an odometry
               message was already received and that therefore cannot be
               associated to a new timestamp).
        """
        vertex_id = "%s_%s" % (node_type, node_id)
        # If no nodes of the input node type were ever associated to a timestamp
        # initialize the first level of the dictionaries.
        if node_type not in self.num_local_indices_assigned:
            self.num_local_indices_assigned[node_type] = dict()
            self.last_time_stamp[node_type] = dict()
        if node_type not in self.timestamp_local_indices:
            self.timestamp_local_indices[node_type] = dict()
        # If the input node was never associated to a timestamp initialize the
        # second level of the dictionaries.
        if node_id not in self.num_local_indices_assigned[node_type]:
            self.num_local_indices_assigned[node_type][node_id] = 0
            self.last_time_stamp[node_type][node_id] = 0.0
        if node_id not in self.timestamp_local_indices[node_type]:
            self.timestamp_local_indices[node_type][node_id] = dict()

        num_local_indices_assigned = self.num_local_indices_assigned[node_type][
            node_id]

        # An odometry node that is already associated to a timestamp is of a
        # movable type (Duckiebots move and therefore send odometry information)
        # and has num_local_indices_assigned != 0.
        is_not_previously_seen_odometry_node = (node_type in self.movable or
                                                num_local_indices_assigned == 0)

        if time_stamp not in self.timestamp_local_indices[node_type][node_id] and is_not_previously_seen_odometry_node:
            # Assign the next available local index to the timestamp and
            # increment the number of local indices assigned.
            self.timestamp_local_indices[node_type][node_id][
                time_stamp] = num_local_indices_assigned
            self.num_local_indices_assigned[node_type][
                node_id] = self.num_local_indices_assigned[node_type][node_id] + 1
            # If the new timestamp is posterior to all previosly-received
            # timestamps simply set it to be the timestamp furthest in time
            # among those associated to the node. If, on the contrary, due to
            # delays in the transmission/reception of the messages this is not
            # the case, perform retro-interpolation in enabled.
            if (time_stamp > self.last_time_stamp[node_type][node_id]):
                self.last_time_stamp[node_type][node_id] = time_stamp
            else:
                if (self.retro_interpolate):
                    retro_interpolate(self, time_stamp, node_type, node_id,
                                      vertex_id)
            return True
        return False

    def add_vertex(self,
                   vertex_id,
                   theta,
                   p,
                   is_initial_floor_tag=False,
                   fixed=False,
                   time_stamp=0.0):
        """Adds a vertex in the graph.

           Args:
               vertex_id:
               theta:
               p:
               is_initial_floor_tag:
               fixed:
               time_stamp:
        """
        [node_type, node_id] = vertex_id.split("_")
        # Adds (assigns) a timestamp to a node, by giving it a 'local index'.
        added = self.add_timestamp_to_node(node_type, node_id, time_stamp)
        if (not added):
            print(
                "add_vertex did not add : node %s at time %f as already there" %
                (vertex_id, time_stamp))
        else:

            p = [p[0], p[1], 0.0]
            R = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), np.deg2rad(theta))
            if (is_initial_floor_tag):
                R2 = g.rotation_from_axis_angle(
                    np.array([1, 0, 0]), np.deg2rad(180))
                R = np.matmul(R, R2)
            # print(R, theta)
            vertex_pose = g2o.Isometry3d(R, p)
            vertex_id = self.convert_names_to_int(vertex_id, time_stamp)
            self.graph.add_vertex(vertex_id, vertex_pose, fixed=fixed)
            if (is_initial_floor_tag):
                self.graph.add_edge(0, vertex_id, vertex_pose)

    def convert_names_to_int(self, id, time_stamp):
        """Given an ID in the format <node_type>_<node_id> and a timestamp
           associated to that node, outputs an integer that can be used as a
           index for the node in g2o (the latter only handles integer indices
           for the nodes).

           Args:
              id: ID in the format <node_type>_<node_id>
              time_stamp: Timestamp in seconds.

           Returns:
              Input ID converted to an integer that can be used as an index by
              g2o.
        """
        [node_type, node_id] = id.split("_")
        a = 0
        b = int(node_id) % 1000

        if (node_type == "duckie"):
            a = 1
        elif (node_type == "watchtower"):
            a = 2
        elif (node_type == "apriltag"):
            a = 3

        if (node_type in self.movable):
            c = self.timestamp_local_indices[node_type][node_id][time_stamp]
            if (c >= 100000):
                print("overflow of the time_stamp list")
        else:
            c = 0

        result = a * 10**8 + b * 10**5 + c
        return result

    def retro_interpolate(self, time_stamp, node_type, node_id, vertex_id):

        sorted_time_stamp = sorted(
            self.timestamp_local_indices[node_type][node_id].keys())
        time_stamp_index = sorted_time_stamp.index(time_stamp)
        if (time_stamp_index > 0 and
                time_stamp_index < len(sorted_time_stamp) - 1):

            before = self.convert_names_to_int(
                vertex_id, sorted_time_stamp[time_stamp_index - 1])
            after = self.convert_names_to_int(
                vertex_id, sorted_time_stamp[time_stamp_index + 1])
            transform = self.graph.get_transform(before, after)
            if (transform != 0):
                print("Will perform retro-interpolation")

                self.interpolate(sorted_time_stamp[time_stamp_index - 1],
                                 sorted_time_stamp[time_stamp_index + 1],
                                 node_type, node_id, transform)
            else:
                print("will not perform retro_interpolation with %d and %d " %
                      (before, after))

    def interpolate(self, old_time_stamp, new_time_stamp, node_type, node_id,
                    measure):
        vertex_id = "%s_%s" % (node_type, node_id)
        to_interpolate = {
            time_stamp:
            self.timestamp_local_indices[node_type][node_id][time_stamp]
            for time_stamp in self.timestamp_local_indices[node_type][
                node_id].keys()
            if (time_stamp >= old_time_stamp and time_stamp <= new_time_stamp)
        }
        # print("interpolating one odometry measure on %d nodes" %
        #       len(to_interpolate))
        sorted_time_stamps = sorted(to_interpolate.keys())

        total_delta_t = float(sorted_time_stamps[-1] - sorted_time_stamps[0])
        if (total_delta_t == 0.0):
            print("in interpolate, delta t is 0.0, with %s %s and list is:" %
                  (node_type, node_id))
            print(to_interpolate)
            print("new_time_stamp is %f and old time stamp is %f" %
                  (old_time_stamp, new_time_stamp))
            print(self.timestamp_local_indices[node_type][node_id])
        R = measure.R
        t = measure.t
        q = g.SE3_from_rotation_translation(R, t)
        vel = g.SE3.algebra_from_group(q)
        check = 0.0
        for i in range(0, len(sorted_time_stamps) - 1):
            partial_delta_t = float(
                sorted_time_stamps[i + 1] - sorted_time_stamps[i])
            alpha = partial_delta_t / total_delta_t
            check += alpha
            rel = g.SE3.group_from_algebra(vel * alpha)
            newR, newt = g.rotation_translation_from_SE3(rel)
            interpolated_measure = g2o.Isometry3d(newR, newt)
            vertex0_id_int = self.convert_names_to_int(vertex_id,
                                                       sorted_time_stamps[i])
            vertex1_id_int = self.convert_names_to_int(
                vertex_id, sorted_time_stamps[i + 1])
            self.graph.add_edge(vertex0_id_int, vertex1_id_int,
                                interpolated_measure)
        # print("end interpolation")
        if (check != 1.0):
            # print("damn, check is %.4f instead of 1.0" % check)
            pass

    def optimize(self,
                 number_of_steps,
                 verbose=True,
                 save_result=True,
                 output_name="output.g2o"):
         """Performs optimization.

            Args:
                number_of_steps: Number of steps of the optimization performed
                                 by g2o.
                verbose: Set to true for detailed prints.
                save_result: Set to true to save the result of the optimization
                             to file.
                output_name: Output filename of the result of the optimization.
         """
        self.graph.optimize(
            number_of_steps,
            verbose=verbose,
            save_result=save_result,
            output_name=output_name)

    def add_edge(self,
                 vertex0_id,
                 vertex1_id,
                 measure,
                 time_stamp,
                 old_time_stamp=0):
        # print(self.timestamp_local_indices)
        if (vertex0_id != vertex1_id):
            for vertex_id in [vertex0_id, vertex1_id]:
                if (len(vertex_id.split("_")) == 1):
                    print("Error, vertexname is %s. Exiting" % vertex_id)
                    exit(-1)
                [node_type, node_id] = vertex_id.split("_")
                added = self.add_timestamp_to_node(node_type, node_id,
                                                   time_stamp)
                if not added:
                    pass
            vertex0_id_int = self.convert_names_to_int(vertex0_id, time_stamp)
            vertex1_id_int = self.convert_names_to_int(vertex1_id, time_stamp)

            self.graph.add_edge(vertex0_id_int, vertex1_id_int, measure)
        else:
            [node_type, node_id] = vertex0_id.split("_")
            added = self.add_timestamp_to_node(node_type, node_id, time_stamp)

            if (node_type in self.movable):
                if (old_time_stamp == 0):
                    old_time_stamp = sorted(self.timestamp_local_indices[
                        node_type][node_id].keys())[0]
                if (old_time_stamp != time_stamp):
                    self.interpolate(old_time_stamp, time_stamp, node_type,
                                     node_id, measure)
                else:
                    self.add_vertex(
                        vertex0_id,
                        0, [0, 0],
                        is_initial_floor_tag=False,
                        fixed=False,
                        time_stamp=time_stamp)
            else:

                print(
                    "Node type %s should be movable if given odometry transform"
                    % node_type)
                exit(-1)

    def get_all_poses(self):
        result_dict = {}
        for node_type, listdict in self.timestamp_local_indices.iteritems():
            result_dict[node_type] = {}
            for node_id, time_stamp_dict in listdict.iteritems():
                last_time_stamp = self.last_time_stamp[node_type][node_id]
                vertex_id = "%s_%s" % (node_type, node_id)
                if (self.convert_names_to_int(vertex_id, last_time_stamp) not in
                        self.graph.optimizer.vertices()):
                    # print("len(time_stamp_dict) = %i for %s %s" %
                    #       (len(time_stamp_dict), node_type, node_id))
                    if (node_type in self.movable):
                        last_time_stamp = sorted(time_stamp_dict.keys())[-2]

                result_dict[node_type][node_id] = self.graph.vertex_pose(
                    self.convert_names_to_int(vertex_id, last_time_stamp))
        # print(result_dict)
        return result_dict
