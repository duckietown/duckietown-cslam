# Based on tutorial in duckietown-world/notebooks.
from matplotlib import pyplot as plt
import networkx as nx

class GraphVisualizer():
    """The following class visualizes a g2o graph with the networkx package.

       Attributes:
           graph: Internal networkx graph.
           weights_on_edges: True or False depending if weights should be added
                             on edges.
    """
    def __init__(self, duckietown_graph_builder_=None):
        # Create directed graph.
        self.graph = nx.DiGraph()
        # If a graph is passed automatically converts it into a networkx graph.
        if duckietown_graph_builder_ is not None:
            self.init_graph_from_duckietown_graph(duckietown_graph_builder_)
        self.weights_on_edges = False

    def add_node(self, node_name):
        """Adds a node to the graph.

           Args:
               node_name: Name of the node to be added in the graph.
        """
        self.graph.add_node(node_name)

    def add_edge(self, node_from_name, node_to_name, weight=None):
        """Adds an edge between two nodes.

           Args:
               node_from_name: Name of the node from which the edge starts.
               node_to_name: Name of the node towards which the edge is
                             directed.
               weight: Weight assigned to the node.
        """
        # Nodes get added automatically => No need to check for their existance
        # in the graph.
        if weight is not None and self.weights_on_edges:
            self.graph.add_edge(node_from_name, node_to_name, weight=weight)
        else:
            self.graph.add_edge(node_from_name, node_to_name)

    def draw_graph(self, duckietown_graph_builder=None):
        """Draws the internal graph or an input graph in matplotlib.

           Args:
               duckietown_graph_builder: DuckietownGraphBuilder that contains
                                         the graph to be set as the internal
                                         graph and printed. If None, just
                                         prints the internal graph.
        """
        if (duckietown_graph_builder is not None):
            self.init_graph_from_duckietown_graph(duckietown_graph_builder)
        plt.figure("Pose graph")
        nx.draw(self.graph, pos=None, labels={node:node for node in self.graph.nodes()})

        def edge_label(node_from, node_to):
            """ Labels an edge from node <node_from> to node <node_to> in the
                plot.
            """
            if self.weights_on_edges:
                edge_data = self.graph.get_edge_data(node_from, node_to)
                return edge_data['weight']
            else:
                return ""

        edge_labels = dict([((node_from, node_to),
                             edge_label(node_from, node_to))
                             for node_from, node_to in self.graph.edges()])
        nx.draw_networkx_edge_labels(self.graph,
                                     pos=nx.spring_layout(self.graph),
                                     edge_labels=edge_labels,
                                     font_color='red')
        plt.axis('off')
        plt.show()

    def init_graph_from_duckietown_graph(self, duckietown_graph_builder):
        """Creates the internal graph from scratch based on an input
           DuckietownGraphBuilder.

           Args:
               duckietown_graph_builder: DuckietownGraphBuilder instance to be
                                         used to create the internal graph.
        """
        # Clear internal graph.
        self.graph.clear()
        # For all edges in the g2o graph add a corresponding edge in the graph
        # using the relative transform as weight.
        graph = duckietown_graph_builder.graph
        node_from = 0
        node_to = 0
        time_stamp_from = 0
        time_stamp_to = 0
        for edge in graph.optimizer.edges():
            node_from = edge.vertices()[0].id()
            node_to = edge.vertices()[1].id()
            # Not an April tag directly connected to the base vertex.
            if (node_from != 0):
                self.add_edge(node_from, node_to)
