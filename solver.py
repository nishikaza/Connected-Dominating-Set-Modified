import networkx as nx
from parse import read_input_file, write_output_file
from utils import is_valid_network, average_pairwise_distance, average_pairwise_distance_fast
from queue import PriorityQueue
from random import sample, shuffle
from dominating_set import *
import sys


def solve(G):
    """Approximates a connected dominating set across a graph input.

    Parameters
    ----------
    G: NetworkX graph

    Returns
    -------
    T: A connected dominating set across G

    Notes
    -----
    This function was designed to be a randomized approximation algorithm. 
    This is an NP-hard problem which makes finding a correct solution computationally difficult.
    However, by employing probabilistic measures, this algorithm can ensure a solution T that approximates the optimal solution T'.

    """

    # initialize variables to track, and eventually return, best tree
    best_distance = float('inf')
    best_tree = G
    curr_tree = G

    # run loop 1000 times for each graph input to continually improve solution over time
    for i in range(1000):
        curr_graph = solve_computation(G)
        curr_distance = average_pairwise_distance_fast(curr_tree)
        if (curr_distance < best_distance):
            best_distance = curr_distance
            best_tree = curr_tree
    return best_tree


def create_initial_tree(G):
    """Approximates a connected dominating set across a graph input.

    Parameters
    ----------
    G: NetworkX graph

    Returns
    -------
    disconnected_dominating_graph: A disconnected NetworkX graph that represents one possible dominating set of G

    Notes
    -----
    This function uses dominating_set from the local file NOT nx.dominating_set. After repeated trials, it was found that
    the randomization processes used by the nx.dominating_set import yielded undesirable results and thus had to be
    modified for this use case.

    """

    tree_set = dominating_set(G)
    disconnected_dominating_graph = nx.Graph()
    for node in list(G.nodes()):
        if node in tree_set:
            disconnected_dominating_graph.add_node(node)
    return disconnected_dominating_graph


def solve_computation(G):
    if G.number_of_nodes == 0:
        return G
    input_graph_copy = G.copy()
    T = create_initial_tree(input_graph_copy)
    vertextrack = set()
    for node in list(T.nodes()):
        vertextrack.add(node)
    while(nx.is_connected(T) == False):
        random_nodes = sample(list(vertextrack), 2)
        shortest_path = nx.dijkstra_path(
            input_graph_copy, random_nodes[0], random_nodes[1])
        for i in range(len(shortest_path)-1):
            T.add_edge(shortest_path[i], shortest_path[i+1],
                       weight=G[shortest_path[i]][shortest_path[i+1]]['weight'])
    counter = 0
    while(nx.is_tree(T) == False):
        counter = counter + 1
        try:
            cycle_edges = nx.find_cycle(T)
            max = 0
            maxEdgeFrom = 0
            maxEdgeTo = 0
            for i in cycle_edges:
                if max < G[i[0]][i[1]]['weight']:
                    max = G[i[0]][i[1]]['weight']
                    maxEdgeFrom = i[0]
                    maxEdgeTo = i[1]
            T.remove_edge(maxEdgeFrom, maxEdgeTo)
        except:
            break
    node_list = list(T.nodes())
    shuffle(node_list)
    if len(node_list) != 1:
        for node in node_list:
            temp_graph = T.copy()
            temp_graph.remove_node(node)
            if (temp_graph.number_of_nodes() != 0 and G.number_of_nodes() != 0):
                if is_valid_network(G, temp_graph):
                    T.remove_node(node)
    return T
