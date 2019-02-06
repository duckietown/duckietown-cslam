import threading

import g2o
import geometry as g
import numpy as np


class g2oGraphBuilder():

    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        # self.solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
        # self.solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
        self.solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())

        self.algorithm = g2o.OptimizationAlgorithmLevenberg(self.solver)
        self.optimizer.set_algorithm(self.algorithm)
        self.already_initialized = False
        self.last_lost = 0
        self.lock = threading.Lock()
        self.set_of_new_edges = set()
        self.set_of_new_vertex = set()
        self.has_removed = False

    def add_vertex(self, vertex_id, vertexPose, fixed=False):
        # print("add_vertex")
        # vertexPose has to be Isometry3D
        vc = g2o.VertexSE3()
        vc.set_id(vertex_id)
        vc.set_estimate(vertexPose)
        vc.set_fixed(fixed)
        
        self.optimizer.add_vertex(vc)
        self.lock.acquire()
        self.set_of_new_vertex.add(vc)
        self.lock.release()

    def add_edge(self, vertex0_id, vertex1_id, measure, measure_information=None, robust_kernel=None):
        '''
        Helper function to add an edge
        vertex 0 and 1 are valid IDs of vertex inside optimizer.
        measure is a Isometry3d that map the transform from vertex0 to vertex1
        '''
        
        ## TODO 
        # robust kernels? They are made to reduce the impact of outliers.
        # They have to be specified separatly for each edge, with a value
        # example : robust_kernel = g2o.RobustKernelHuber(np.sqrt(5.991))
        #           edge.set_robust_kernel(robust_kernel)
        ##

        # print("add_edge")
        # print(optimizer.vertices())
        if vertex0_id in self.optimizer.vertices():
            if vertex1_id not in self.optimizer.vertices():
                vc1 = g2o.VertexSE3()
                vc1.set_id(vertex1_id)
                vc1.set_estimate(
                    self.optimizer.vertex(vertex0_id).estimate() * measure)
                vc1.set_fixed(False)
                self.optimizer.add_vertex(vc1)
                self.lock.acquire()
                self.set_of_new_vertex.add(vc1)
                self.lock.release()
            edge = g2o.EdgeSE3()
            edge.set_vertex(0, self.optimizer.vertex(vertex0_id))
            edge.set_vertex(1, self.optimizer.vertex(vertex1_id))
            edge.set_measurement(measure)

            if measure_information is not None:
                edge.set_information(measure_information)
            
            if robust_kernel is not None:
                edge.set_robust_kernel(robust_kernel)

            self.optimizer.add_edge(edge)

            # Adding edge to set of new edge to update initialization
            self.lock.acquire()
            self.set_of_new_edges.add(edge)
            self.lock.release()


        else:
            if vertex1_id in self.optimizer.vertices():
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0_id)
                vc0.set_estimate(
                    self.optimizer.vertex(vertex1_id).estimate() *
                    measure.inverse())
                vc0.set_fixed(False)
                self.optimizer.add_vertex(vc0)
                self.lock.acquire()

                self.set_of_new_vertex.add(vc0)
                self.lock.release()
                edge = g2o.EdgeSE3()
                edge.set_vertex(0, self.optimizer.vertex(vertex0_id))
                edge.set_vertex(1, self.optimizer.vertex(vertex1_id))
                edge.set_measurement(measure)
                if (measure_information):
                    edge.set_information(measure_information)
                # edge.set_information(np.eye(6) * 2)
                finished = self.optimizer.add_edge(edge)
                self.lock.acquire()
                self.set_of_new_edges.add(edge)
                self.lock.release()

            else:
                vc0 = g2o.VertexSE3()
                vc0.set_id(vertex0_id)
                vc0.set_fixed(False)
                self.optimizer.add_vertex(vc0)
                self.lock.acquire()

                self.set_of_new_vertex.add(vc0)
                self.lock.release()

                self.add_edge(vertex0_id, vertex1_id, measure)

    def vertices_and_edges(self):
        return (self.optimizer.vertices(), self.optimizer.edges())

    def vertex_pose(self, vertexId):
        # print("vertex_pose")

        if (vertexId not in self.optimizer.vertices()):
            print("Vertex %i is not in the g2o graph" % vertexId)
            if (self.last_lost != 0 and
                    self.last_lost in self.optimizer.vertices()):
                print(
                    "Vertex %i wasn't but is now in g2o graph" % self.last_lost)
            elif (self.last_lost != 0):
                print("Vertex %i wasn't and still isn't in g2o graph" %
                      self.last_lost)
            self.last_lost = vertexId
        return (self.optimizer.vertex(vertexId).estimate())

    def get_transform(self, vertex1, vertex2):
        # print("get_transform")

        if (vertex1 not in self.optimizer.vertices() or
                vertex2 not in self.optimizer.vertices()):
            print("Requesting transform between non existant vertices")
            return 0
        vc1 = self.optimizer.vertex(vertex1).estimate()
        vc2 = self.optimizer.vertex(vertex2).estimate()
        transfom = vc1.inverse() * vc2
        return transfom

    def remove_vertex(self, vertex_id):
        # print("remove_vertex")

        self.lock.acquire()
        self.has_removed = True
        # print("trying to remove %i" % vertex_id)
        if(vertex_id in self.optimizer.vertices()):
            vc = self.optimizer.vertex(vertex_id)
            # print("it is here, let's remove it")
            if(vc in self.set_of_new_vertex):
                # print("removing %i or the set of new vertex" % vertex_id)
                self.set_of_new_vertex.remove(vc)
            edge_to_remove = []
            for edge in self.set_of_new_edges:
                # print(edge.vertices())
                if vc in edge.vertices():
                    # print("edges to remove added edge")
                    if edge not in edge_to_remove:
                        edge_to_remove.append(edge)
            for edge in edge_to_remove:
                self.set_of_new_edges.remove(edge)
            self.optimizer.remove_vertex(vc, detach=True)

            # print("success")
        else:
            print("was not in optimizer")
        self.lock.release()

    def optimize(self,
                 number_of_steps,
                 verbose=True,
                 save_result=True,
                 output_name="output.g2o",
                 online=False):
        # print("optimize")

        self.lock.acquire()

        self.optimizer.set_verbose(verbose)
        if (not self.already_initialized):

            self.optimizer.initialize_optimization()
            self.set_of_new_edges = set()
            self.set_of_new_vertex = set()
            self.already_initialized = True
        else:
            if(not self.has_removed):
                self.optimizer.update_initialization(self.set_of_new_vertex,
                                                     self.set_of_new_edges)
            else:
                self.optimizer.initialize_optimization()
                self.has_removed = False
            self.set_of_new_edges = set()
            self.set_of_new_vertex = set()
            self.optimizer.compute_initial_guess()
        self.optimizer.compute_active_errors()
        if (verbose):
            print('Optimization:')
            print('Initial chi2 = %f' % self.optimizer.chi2())
        self.optimizer.optimize(number_of_steps, online=online)
        
        if (save_result):
            self.optimizer.save(output_name)
        
        self.lock.release()
        
        return self.optimizer.chi2()
        # batch_stat = self.optimizer.batch_statistics
        # print(batch_stat)
