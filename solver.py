import networkx as nx
from parse import read_input_file, write_output_file
from utils import is_valid_network, average_pairwise_distance, average_pairwise_distance_fast
from queue import PriorityQueue
from random import sample, shuffle
from dominating_set import *
import sys


def solve(G):
    """Approximates a connected dominating set across a graph input by selecting the optimal solution from many randomly generated candidate solutions.

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
        curr_tree = solve_computation(G)
        curr_distance = average_pairwise_distance_fast(curr_tree)
        if (curr_distance < best_distance):
            best_distance = curr_distance
            best_tree = curr_tree
    return best_tree


def create_initial_tree(G):
    """Approximates a disconnected dominating set across a graph input.

    Parameters
    ----------
    G: NetworkX graph

    Returns
    -------
    disconnected_dominating_graph: A disconnected NetworkX graph that represents one possible dominating set of G

    Notes
    -----
    This function uses dominating_set from the local file, NOT from nx.dominating_set. After repeated trials, it was found that
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
    """Approximates a connected dominating set across a graph input.

    Parameters
    ----------
    G: NetworkX graph

    Returns
    -------
    T: A connected dominating set across G

    Notes
    -----
    This function utilizes randomness to generate a single candidate solution for the connected dominating set problem.
    This function as well as all functions called within it are run many times to generate the aforementioned candidate solutions.

    """

    if G.number_of_nodes == 0:
        return G
    disconnected_dom_graph = create_initial_tree(G)
    connected_dom_graph = find_shortest_paths(disconnected_dom_graph, G)
    connected_no_cycles = eliminate_cycles(connected_dom_graph, G)
    final_graph = remove_repeat_nodes(connected_no_cycles)
    return final_graph


def find_shortest_paths(G, original_graph):
    """Connects a disconnected dominating set by finding shortest paths between vertices in the 
    approximate disconnected dominating set.

    Parameters
    ----------
    G: NetworkX graph containing an approximate dominating set
    original_graph: NetworkX graph containing the original input graph and all of its edges

    Returns
    -------
    T: A connected dominating set across original_graph

    Notes
    -----
    This function randomly chooses vertices across which it finds a shortest path until the graph is connected. 
    After solve(G) is run many times on each graph input, the outputted connected dominating set was found to be greatly improved.

    """
    vertextrack = set()
    for node in list(G.nodes()):
        vertextrack.add(node)
    while(nx.is_connected(G) == False):
        random_nodes = sample(list(vertextrack), 2)
        shortest_path = nx.dijkstra_path(
            original_graph, random_nodes[0], random_nodes[1])
        for i in range(len(shortest_path)-1):
            G.add_edge(shortest_path[i], shortest_path[i+1],
                       weight=G[shortest_path[i]][shortest_path[i+1]]['weight'])
    return G


def eliminate_cycles(G, original_graph):
    """Removes all cycles in an approximate connected dominating set.

    Parameters
    ----------
    G: NetworkX graph containing an approximate connected dominating set
    original_graph: NetworkX graph containing the original input graph and all of its edges

    Returns
    -------
    T: A connected, acyclic dominating set across original_graph

    """

    while(nx.is_tree(G) == False):
        try:
            cycle_edges = nx.find_cycle(T)
            max = 0
            maxEdgeFrom = 0
            maxEdgeTo = 0
            for i in cycle_edges:
                if max < original_graph[i[0]][i[1]]['weight']:
                    max = original_graph[i[0]][i[1]]['weight']
                    maxEdgeFrom = i[0]
                    maxEdgeTo = i[1]
            G.remove_edge(maxEdgeFrom, maxEdgeTo)
        except:
            break
    return G


def remove_repeat_nodes(G):
    """Removes all unnecessary nodes in an approximate connected dominating set.

    Parameters
    ----------
    G: NetworkX graph containing an approximate connected dominating set

    Returns
    -------
    T: A connected, acyclic dominating set across original_graph

    """

    node_list = list(G.nodes())
    shuffle(node_list)
    if len(node_list) != 1:
        for node in node_list:
            temp_graph = G.copy()
            temp_graph.remove_node(node)
            if (temp_graph.number_of_nodes() != 0 and G.number_of_nodes() != 0):
                if is_valid_network(G, temp_graph):
                    G.remove_node(node)
    return G
