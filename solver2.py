import networkx as nx
from parse import read_input_file, write_output_file
from utils import is_valid_network, average_pairwise_distance, average_pairwise_distance_fast
from queue import PriorityQueue
from random import sample, shuffle
import sys


def create_initial_tree(G):
    tree_set = nx.dominating_set(G)
    T = nx.Graph()
    for node in list(G.nodes()):
        if node in tree_set:
            T.add_node(node)
    return T


def solve_helper(G):
    if G.number_of_nodes == 0:
        return G
    q = G.copy()
    T = create_initial_tree(q)
    p = G.copy()
    vertextrack = set()
    for node in list(T.nodes()):
        vertextrack.add(node)
    while(nx.is_connected(T) == False):
        random_nodes = sample(list(vertextrack), 2)
        shortest_path = nx.dijkstra_path(p, random_nodes[0], random_nodes[1])
        for i in range(len(shortest_path)-1):
            T.add_edge(shortest_path[i], shortest_path[i+1],
                       weight=G[shortest_path[i]][shortest_path[i+1]]['weight'])
    counter = 0
    while(nx.is_tree(T) == False):
        counter = counter+1
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
    node_list = list(T.nodes())
    shuffle(node_list)
    if len(node_list) != 1:
        for node in node_list:
            temp_graph = T.copy()
            temp_graph.remove_node(node)
            if is_valid_network(G, temp_graph):
                T.remove_node(node)
    return T


def solve(G):
    best_distance = 100000000
    best_graph = G
    curr_graph = G

    for i in range(1000):
        curr_graph = solve_helper(G)
        curr_distance = average_pairwise_distance_fast(curr_graph)
        if (curr_distance < best_distance):
            best_distance = curr_distance
            best_graph = curr_graph
        if i % 100 == 0:
            print(best_distance)
    return best_graph
# Pick vertices in descending order of degree until all vertices are either in or adjacent from T
# Randomly choose pairs of vertices from T and calculate the shortest path between them. Add these edges to T
# Halt when T is connected, return T


# Here's an example of how to run your solver.

# Usage: python3 solver.py test.in

if __name__ == '__main__':
    large = "large-"
    for i in range(76, 401):
        print(i)
        curLarge = large + str(i) + ".in"
        G = read_input_file("inputs/"+curLarge)
        T = solve(G.copy())
        assert is_valid_network(G, T)
        write_output_file(T, 'outputs/' + curLarge[0:-3] + '.out')
    print("done with large")
