import numpy as np
import g2o
import duckietown_cslam.g2oGraphBuilder.g2ograph_builder as g2oBG
import geometry as g
import random
import yaml

time_step = 0.5


class DuckietownGraphBuilder():
    """Creates an internal g2o pose graph, and optimizes over it. At the same
       time, it keeps track of the timestamps at which messages related to an
       object (e.g. watchtower, Duckiebot) were received. We refer to objects as
       "nodes", with each node being identified by the ID of its April tag
       (node_id) and by its type (node_type). Types are defined in the class
       constructor and could e.g. be Duckiebots, watchtowers, etc.
       We instead refer to the nodes in the pose graph as "vertices".

       Attributes:
           graph: Pose graph (g2oGraphBuilder object).
           types: Possible types of nodes in the graph, i.e., of objects in
                  Duckietown.
           timestamp_local_indices:
                  Dictionary of dictionaries of dictionaries that keeps track of
                  the timestamps for each node, for each type of node. In
                  particular, a certain vertex in the graph represents an object
                  at a certain instant of time (timestamp) and is identified in
                  the internal g2o representation with a strictly integer index.
                  Since each node (i.e., object) can be seen or see another
                  node several times, multiple vertices are added to the graph
                  for the same node, with each vertex identifying the node at a
                  specific timestamp. To define the integer index that g2o needs
                  to associate to each vertex, we define an encoding (cf.
                  convert_names_to_int) based on the type of the node, the ID of
                  the node and the timestamp. Furthermore, to compress the
                  representation, for the encoding we do not use the timestamp
                  itself but rather a 'local index' that defines the order in
                  which new timestamps are associated to a node. For instance,
                  if a Duckiebot (node_type="duckie", node_id=88) first sees an
                  April tag and sends the corresponding message at timestamp t1,
                  the latter timestamp is assigned 'local index' 0 for that
                  node. If the Duckiebot is later detected by a watchtower, with
                  message sent at timestamp t2 > t1, timestamp t2 is assigned
                  'local index' 1 for the node.
                  The 'local index' given to the timestamp <time_stamp> (float)
                  w.r.t. the node with <node_id> and type <node_type> is
                  stored in
                  timestamp_local_indices[<node_type>][<node_id>][<time_stamp>].
           movable: List of the types of nodes that, for the purposes of
                    optimization, should be considered as objects that can move
                    in Duckietown (pose can be optimized).
           num_local_indices_assigned:
                    Dictionary of dictionaries.
                    num_local_indices_assigned[<node_type>][<node_id>] stores
                    the number of 'local indices' already assigned to timestamps
                    by the node of type <node_type> and with ID <node_id>.
           last_time_stamp: Dictionary of dictionaries.
                            last_time_stamp[<node_type>][<node_id>] contains the
                            timestamp latest in time among those associated to
                            the node of type <node_type> and with ID <node_id>.
           last/first_odometry_time_stamp:
                    Dictionaries of dictionaries.
                    last/first_odometry_time_stamp[<node_type>][<node_id>]
                    contains the timestamp at which the last/first odometry
                    message for the node with type <node_type> and ID <node_id>
                    was received. This information is kept track of to ensure
                    that retro-interpolation is only performed between the
                    first and the last odometry message.
           retro_interpolate: True if retro-interpolation should be performed,
                              False otherwise. Retro-interpolation allows to
                              handle messages with a timestamp anterior to the
                              latest timestamp of the already-processed
                              messages. See retrointerpolate method for further
                              details.
           chi2 : The measure of the uncertainty of the graph, given by the optimizer in g2o
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
        # Initialize first-level dictionary of first odometry time stamps.
        self.first_odometry_time_stamp = dict()
        # Initialize first-level dictionary of last odometry time stamps.
        self.last_odometry_time_stamp = dict()
        # Initialize chi2
        self.chi2 = 0.0
        # Set retro-interpolate mode as inputted.
        self.retro_interpolate = retro_interpolate
       # Load the initial floor April tags if given an input file name
        if (initial_floor_april_tags != ""):
            self.load_initial_floor_april_tags(initial_floor_april_tags)

    def load_initial_floor_april_tags(self, initial_floor_april_tag_file):
        """Adds the poses of the initial floor April tags to the graph by
           reading them from file.

           Args:
               initial_floor_april_tag_file: File from where to read the poses
                                             of the floor tags.
        """
        # TODO: Use duckietown-world to load the tags.
        with open(initial_floor_april_tag_file, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                # Add a reference vertex to the g2o graph.
                vertex_pose = g2o.Isometry3d(np.eye(3), [0, 0, 0])
                self.graph.add_vertex(0, vertex_pose, fixed=True)
                # Add vertices for all the floor tags.
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
                                    is_initial_floor_tag=True,
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
               time_stamp: Timestamp.

           Returns:
               True if the operation is successful, False otherwise (i.e., if
               the timestamp is already associated to the node with a 'local
               index' or if the the node is not movable and already has
               an associated timestamp).
        """
        vertex_id = "%s_%s" % (node_type, node_id)
        # If no nodes of the input node type were ever associated to a timestamp
        # initialize the first level of the dictionaries.
        if node_type not in self.num_local_indices_assigned:
            self.num_local_indices_assigned[node_type] = dict()
            self.last_time_stamp[node_type] = dict()
            self.last_odometry_time_stamp[node_type] = dict()
            self.first_odometry_time_stamp[node_type] = dict()
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

        if (time_stamp not in self.timestamp_local_indices[node_type][node_id]
                and
                (node_type in self.movable or num_local_indices_assigned == 0)):
            # Assign the next available local index to the timestamp and
            # increment the number of local indices assigned.
            self.timestamp_local_indices[node_type][node_id][
                time_stamp] = num_local_indices_assigned
            self.num_local_indices_assigned[node_type][
                node_id] = self.num_local_indices_assigned[node_type][node_id] + 1
            # If the new timestamp is posterior to all previously-received
            # timestamps simply set it to be the timestamp furthest in time
            # among those associated to the node. If, on the contrary, due to
            # delays in the transmission/reception of the messages this is not
            # the case, perform retro-interpolation if enabled.
            if (time_stamp > self.last_time_stamp[node_type][node_id]):
                self.last_time_stamp[node_type][node_id] = time_stamp
            else:
                if (self.retro_interpolate and node_type in self.movable):
                    # Check that message is in the odometry chained part of the graph
                    if(node_id in self.last_odometry_time_stamp[node_type] and node_id in self.first_odometry_time_stamp[node_type]):
                        if(time_stamp > self.first_odometry_time_stamp[node_type][node_id] and
                           time_stamp < self.last_odometry_time_stamp[node_type][node_id]):
                            # check that optimization was made at least once, so that no absurd edge is created
                            if(self.chi2 != 0.0):
                                self.retrointerpolate(time_stamp, node_type, node_id,
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
        """Adds a vertex with an ID given in the format outputted from the
          transform_listener and with a given pose to the g2o graph.

TODO : add a more general add_vertex function that takes a 3D pose and not only a 2D one.

           Args:
               vertex_id: ID of the node whose pose should be added as a vertex
                          in the graph, in the format <node_type>_<node_id>.
               theta: Angle of the 2D rotation around the z axis defines the
                      pose of the node.
               p: 2D translation vector (x, y) that define the pose of the node.
               is_initial_floor_tag: True if the node is an initial floor tag,
                                     False otherwise.
               fixed: True if the node should be added as a fixed vertex (i.e.,
                      not optimizable) in the graph, False otherwise.
               time_stamp: Timestamp.
        """
        [node_type, node_id] = vertex_id.split("_")
        # Adds (assigns) a timestamp to a node, by giving it a 'local index'
        # w.r.t. the node.
        added = self.add_timestamp_to_node(node_type, node_id, time_stamp)
        if (not added):
            print(
                "add_vertex did not add : node %s at time %f as already there" %
                (vertex_id, time_stamp))
        else:
            # Translation vector, z coordinate does not vary.
            p = [p[0], p[1], 0.0]
            # Rotation around the z axis of and angle theta.
            R = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), np.deg2rad(theta))
            if (is_initial_floor_tag):
                # For initial floor tags, apply a further rotation of 180
                # degrees around the x axis, so that the orientation of the z
                # axis is reverted.
                R2 = g.rotation_from_axis_angle(
                    np.array([1, 0, 0]), np.deg2rad(180))
                R = np.matmul(R, R2)
            # Add vertex with pose and ID in the right format to the g2o graph.
            vertex_pose = g2o.Isometry3d(R, p)
            vertex_id = self.convert_names_to_int(vertex_id, time_stamp)
            self.graph.add_vertex(vertex_id, vertex_pose, fixed=fixed)
            if (is_initial_floor_tag):
                # For initial floor tags, add an edge between the reference
                # vertex and the newly-added vertex for the pose of the tag.
                self.graph.add_edge(
                    vertex0_id=0, vertex1_id=vertex_id, measure=vertex_pose)

    def convert_names_to_int(self, id, time_stamp):
        """Given an ID in the format <node_type>_<node_id> and a timestamp
           associated to that node, outputs an integer that can be used as a
           index for the node in g2o (the latter only handles integer indices
           for the nodes).

           Args:
              id: ID in the format <node_type>_<node_id>
              time_stamp: Timestamp.

           Returns:
              Input ID converted to an integer that can be used as an index by
              g2o.
        """
        [node_type, node_id] = id.split("_")
        b = int(node_id) % 1000
        a = self.types.index(node_type) + 1

        if (node_type in self.movable):
            c = self.timestamp_local_indices[node_type][node_id][time_stamp]
            if (c >= 100000):
                print("overflow of the time_stamp list")
        else:
            c = 0

        result = a * 10**8 + b * 10**5 + c
        return result

    def retrointerpolate(self, time_stamp, node_type, node_id, vertex_id):
        """Due to delays in the communication network, it might happen that a
           message with timestamp time_stamp is received after a message with
           timestamp time_stamp2, with time_stamp < time_stamp2. If this is the
           case, this function inserts a vertex with timestamp time_stamp by
           interpolating between the two timestamps - among those that have
           already a vertex in the graph - immediately before and
           immediately after time_stamp. This process is referred to as
           retro-interpolation. Note: if time_stamp happens to follow
           or precede in time all the timestamps with a vertex in the graph,
           retro-interpolation cannot be performed.

           Args:
               time_stamp: Timestamp
               node_type: Type of the node.
               node_id: ID of the node.
               vertex_id: ID in the format <node_type>_<node_id>.
        """
        # Find timestamps immediately before and immediately after the given
        # timestamp.
        time_stamp_before = 0.0
        time_stamp_after = float('inf')
        for time_stamp_it in self.timestamp_local_indices[node_type][
                node_id].keys():
            if time_stamp_it < time_stamp:
                if time_stamp_before < time_stamp_it:
                    time_stamp_before = time_stamp_it

            if time_stamp < time_stamp_it:
                if time_stamp_it < time_stamp_after:
                    time_stamp_after = time_stamp_it

        # If the timestamp is neither the first nor the last (i.e., both
        # timestamp before and timestamp after exist) we can go ahead.
        if (time_stamp_before != 0.0 and time_stamp_after != float('inf')):
            before = self.convert_names_to_int(vertex_id, time_stamp_before)
            after = self.convert_names_to_int(vertex_id, time_stamp_after)
            transform = self.graph.get_transform(before, after)
            # If the timestamps before/after have corresponding vertices in the
            # graph perform retro-interpolation.
            if (transform != 0):
                print("Will perform retro-interpolation")

                self.interpolate(time_stamp_before, time_stamp_after, node_type,
                                 node_id, transform)
            else:
                print("will not perform retro_interpolation with %d and %d " %
                      (before, after))

    def interpolate(self, old_time_stamp, new_time_stamp, node_type, node_id,
                    measure):
        """Given a timestamp new_time_stamp at which an odometry message was
           received and the timestamp old_time_stamp of the last odometry
           message previously received, it might be the case that other
           messages, of non-odometry type, have been received in the time
           interval between these two timestamps. For instance, a Duckiebot
           might be seen by a watchtower at two timestamps time_stamp1 and
           time_stamp2 s.t. old_time_stamp < time_stamp1 < time_stamp2 <
           new_time_stamp. This function creates edges in the graph between each
           pair of vertices at consecutive timestamps (e.g.
           old_time_stamp-->time_stamp1, time_stamp1-->time_stamp2,
           time_stamp2-->new_time_stamp). The relative transform between each
           pair of vertices is assigned by performing a linear interpolation in
           the Lie algebra, based on the transform between old_time_stamp and
           new_time_stamp (contained in the odometry message) and on the
           relative time difference (e.g. time_stamp2 - time_stamp1).

           Args:
               old_time_stamp: Timestamp of the last odometry message that was
                               received before the current odometry message, for
                               the node of type node_type and ID node_id.
               new_time_stamp: Timestamp of the current odometry message.
               node_type: Type of the node.
               node_id: ID of the node.
               measure: Transform contained in the current odometry message,
                        between timestamp old_time_stamp and time_stamp
                        new_time_stamp.
        """
        vertex_id = "%s_%s" % (node_type, node_id)
        # Timestamps for which the interpolation should be performed. Note: also
        # old_time_stamp and new_time_stamp are included.
        to_interpolate = {
            time_stamp:
            self.timestamp_local_indices[node_type][node_id][time_stamp]
            for time_stamp in self.timestamp_local_indices[node_type][
                node_id].keys()
            if (time_stamp >= old_time_stamp and time_stamp <= new_time_stamp)
        }
        # Sort the time stamps.
        sorted_time_stamps = sorted(to_interpolate.keys())
        # Find the total time (time between the last and the first timestamp).
        total_delta_t = float(sorted_time_stamps[-1] - sorted_time_stamps[0])
        if (total_delta_t == 0.0):
            print("in interpolate, delta t is 0.0, with %s %s and list is:" %
                  (node_type, node_id))
            print(to_interpolate)
            print("new_time_stamp is %f and old time stamp is %f" %
                  (old_time_stamp, new_time_stamp))
            print(self.timestamp_local_indices[node_type][node_id])
        # Perform a linear interpolation in the Lie algebra associated to SE3
        # group defined by the transform.
        R = measure.R
        t = measure.t
        q = g.SE3_from_rotation_translation(R, t)
        vel = g.SE3.algebra_from_group(q)
        cumulative_alpha = 0.0
        for i in range(0, len(sorted_time_stamps) - 1):
            # Find the time interval between each timestamp and the subsequent
            # one and linearly interpolate accordingly in the algebra.
            partial_delta_t = float(
                sorted_time_stamps[i + 1] - sorted_time_stamps[i])
            alpha = partial_delta_t / total_delta_t
            cumulative_alpha += alpha
            rel = g.SE3.group_from_algebra(vel * alpha)
            newR, newt = g.rotation_translation_from_SE3(rel)
            interpolated_measure = g2o.Isometry3d(newR, newt)
            vertex0_id_int = self.convert_names_to_int(vertex_id,
                                                       sorted_time_stamps[i])
            vertex1_id_int = self.convert_names_to_int(
                vertex_id, sorted_time_stamps[i + 1])
            # Add an edge to the graph, connecting each timestamp to the one
            # following it and using the relative transform obtained by
            # interpolation.
            self.graph.add_edge(vertex0_id_int, vertex1_id_int,
                                interpolated_measure)
        if (cumulative_alpha != 1.0):
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
        self.clean_graph()

        self.chi2 = self.graph.optimize(
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
        """Adds edge between two vertices to the graph.

           Args:
               vertex0/1_id: IDs of the vertices associated to the edge that
                             should be added. For instance the vertices might
                             correspond to the poses of a watchtower and of a
                             tag seen by the watchtower.
               measure: Transform ('measurement' in g2o) associated to the
                        message (and therefore to the edge added to the graph).
               time_stamp: Timestamp of the message that is associated to the
                           edge added to the graph.
               old_time_stamp: Needed for odometry messages. Contains the
                               timestamp of the last odometry message previously
                               received by the object with the given ID (which
                               for odometry messages is either vertex0_id or
                               vertex1_id, since the two match).
        """
        if (vertex0_id != vertex1_id):
            for vertex_id in [vertex0_id, vertex1_id]:
                if (len(vertex_id.split("_")) == 1):
                    print("Error, vertexname is %s. Exiting" % vertex_id)
                    exit(-1)
                [node_type, node_id] = vertex_id.split("_")
                # Associate timestamps to the vertices.
                added = self.add_timestamp_to_node(node_type, node_id,
                                                   time_stamp)
                if not added:
                    pass
            # Obtain ID of the vertices in the graph in the integer format.
            vertex0_id_int = self.convert_names_to_int(vertex0_id, time_stamp)
            vertex1_id_int = self.convert_names_to_int(vertex1_id, time_stamp)
            # Add edge between the two vertices (and automatically also add the
            # vertices if not there already).in the g2o graph.
            self.graph.add_edge(vertex0_id_int, vertex1_id_int, measure)
        else:
            [node_type, node_id] = vertex0_id.split("_")
            # Associate timestamps to the vertex.
            added = self.add_timestamp_to_node(node_type, node_id, time_stamp)

            if (node_type in self.movable):
                # Odometry edge: same vertices, movable object.
                if (old_time_stamp == 0):
                    # No previous odometry messages -> Update old timestamp for
                    # the node to be the oldest timestamp received..
                    print("timestamp_local_indices[node_type={0}][node_id={1}] "
                          "contains the following {2} timestamps:".format(
                              node_type, node_id,
                              len(self.timestamp_local_indices[node_type][
                                  node_id].keys())))
                    for timestamp in self.timestamp_local_indices[node_type][
                            node_id].keys():
                        print(timestamp)
                    print("The current timestamp is {}".format(time_stamp))
                    old_time_stamp = sorted(self.timestamp_local_indices[
                        node_type][node_id].keys())[0]
                    # Initialize the first and last timestamp at which an odometry message for
                    # the node was received to be respectively old_time_stamp and the current
                    # timestamp (time_stamp).
                    self.first_odometry_time_stamp[node_type][node_id] = old_time_stamp
                    self.last_odometry_time_stamp[node_type][node_id] = time_stamp

                if (old_time_stamp != time_stamp):
                    # Update the known last odometry message time_stamp for the node
                    self.last_odometry_time_stamp[node_type][node_id] = time_stamp

                    # Some other messages might have been received for that node
                    # (object), e.g. a Duckiebot might be seen by a watchtower
                    # before receiving an odometry message => Interpolate.
                    self.interpolate(old_time_stamp, time_stamp, node_type,
                                     node_id, measure)
                else:
                    # Add vertex corresponding to the new timestamp to the
                    # graph.
                    self.add_vertex(
                        vertex_id=vertex0_id,
                        theta=0,
                        p=[0, 0],
                        is_initial_floor_tag=False,
                        fixed=False,
                        time_stamp=time_stamp)
            else:
                print(
                    "Node type %s should be movable if given odometry transform"
                    % node_type)
                exit(-1)

    def remove_vertex(self, node_type, node_id, time_stamp):
        vertex_id = "%s_%s" % (node_type, node_id)
        self.graph.remove_vertex(
            self.convert_names_to_int(vertex_id, time_stamp))

    def clean_graph(self):
        """
            Gets rid of useless edges in the graph
            Considered as useless are edges that are anterior to the first odometry message
        """
        for node_type in self.movable:
            for node_id in self.timestamp_local_indices[node_type].keys():
                if(node_id in self.first_odometry_time_stamp[node_type]):
                    first_odometry_time_stamp = self.first_odometry_time_stamp[node_type][node_id]
                    anterior_time_stamps = [time_stamp for time_stamp in self.timestamp_local_indices[node_type][node_id].keys(
                    ) if time_stamp < first_odometry_time_stamp]
                    for time_stamp in anterior_time_stamps:
                        self.remove_vertex(node_type, node_id, time_stamp)
                        self.timestamp_local_indices[node_type][node_id].pop(
                            time_stamp)

    def get_all_poses(self):
        """Obtains all poses in the graph.

           Returns:
               Dictionary of dictionaries, containing for each type of node and
               each node ID the pose corresponding to the latest timestamp
               available in the graph. For instance,
               result_dict[node_type][node_id] contains the latest pose
               available in the g2o graph for the node with type <node_type> and
               ID <node_id>.
        """
        result_dict = {}
        # For all node types <node_type>:
        # timestamp_local_indices[<node_type>] = {<node_id0> : {node_id0_dict},
        #                                         <node_id1> : {node_id1_dict},
        #                                         ...} =: node_id_dict
        for node_type, node_id_dict in self.timestamp_local_indices.iteritems():
            result_dict[node_type] = {}
            # For all node types <node_type>, all node IDs <node_id>:
            # timestamp_local_indices[<node_type>][<node_id>] =
            #     {<time_stamp0> : local_index_of_time_stamp0_in_node_node_id,
            #      <time_stamp0> : local_index_of_time_stamp1_in_node_node_id,
            #      ...} =: time_stamp_dict
            for node_id, time_stamp_dict in node_id_dict.iteritems():
                # Get timestamp furthest in time for node with ID node_id.
                last_time_stamp = self.last_time_stamp[node_type][node_id]
                vertex_id = "%s_%s" % (node_type, node_id)
                if (self.convert_names_to_int(vertex_id, last_time_stamp) not in
                        self.graph.optimizer.vertices()):
                    if (node_type in self.movable):
                        # For odometry nodes that
                        last_time_stamp = sorted(time_stamp_dict.keys())[-2]

                result_dict[node_type][node_id] = self.graph.vertex_pose(
                    self.convert_names_to_int(vertex_id, last_time_stamp))

        return result_dict
