import networkx as nx
from parse import read_input_file, write_output_file
from utils import is_valid_network, average_pairwise_distance, average_pairwise_distance_fast
from queue import PriorityQueue
from random import sample
import sys

def create_initial_tree(G):
    T = nx.Graph()
    heap = PriorityQueue()
    nodetrack = set()
    
    for node in list(G.nodes()):
        nodetrack.add(node)
        
    for (vertex, degree) in G.degree():
        heap.put((-1 * degree, vertex))

    while (nx.number_of_nodes(G) != 0 or heap.qsize() != 0):
        # print("number of nodes: ", nx.number_of_nodes(G))
        # print("nodetrack:", len(nodetrack))
        # print("heap size: ", heap.qsize())
        current_max = heap.get()
        # print("current max: ", current_max[1])

        if (current_max[1] in nodetrack):
            neighbors = list(nx.neighbors(G, current_max[1]))
            T.add_node(current_max[1])
            G.remove_node(current_max[1])
            nodetrack.remove(current_max[1])
            for neighbor in neighbors:
                if (G.has_node(neighbor)):
                    G.remove_node(neighbor)
                    nodetrack.remove(neighbor)
    return T
    
def solve_helper(G):
    """
    Args:
        G: networkx.Graph

    Returns:
        T: networkx.Graph
    """

    q = G.copy()
    T = create_initial_tree(q)
    if G.number_of_nodes == 0:
        return T
    # print("Original number_of_nodes: ", T.number_of_nodes())
    
    p = G.copy()
    vertextrack = set()
    for node in list(T.nodes()):
        vertextrack.add(node)
    while(nx.is_connected(T) == False):
        random_nodes = sample(list(vertextrack), 2)
        shortest_path = nx.dijkstra_path(p,random_nodes[0],random_nodes[1])
        for i in range(len(shortest_path)-1):
            T.add_edge(shortest_path[i],shortest_path[i+1], weight = G[shortest_path[i]][shortest_path[i+1]]['weight'])
    counter = 0
    while(nx.is_tree(T) == False ):
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
    # print("Number of edges: ", len(T.edges()))
    # print("Number of nodes: ", T.number_of_nodes())
    return T    
   
def solve(G):
    best_pairwise_distance = 10000000
    best_graph = G
    for i in range(1000):
        T = solve_helper(G)
        curr_pairwise_distance = average_pairwise_distance_fast(T)
        if (curr_pairwise_distance < best_pairwise_distance):
            best_pairwise_distance = curr_pairwise_distance
            best_graph = T
    return best_graph        
# Pick vertices in descending order of degree until all vertices are either in or adjacent from T
# Randomly choose pairs of vertices from T and calculate the shortest path between them. Add these edges to T
# Halt when T is connected, return T


# Here's an example of how to run your solver.

# Usage: python3 solver.py test.in

if __name__ == '__main__':
    large = "large-"
    for i in range(1,401):
        print(i)
        curLarge = large + str(i) +".in"
        G = read_input_file("inputs/"+curLarge)
        T = solve(G.copy())
        assert is_valid_network(G, T)
        write_output_file(T, 'outputs/' + curLarge[0:-3] + '.out')
        print("done with large")
    medium = "medium-"
    for i in range(1,304):
        print(i)
        curMedium = medium + str(i) +".in"
        G = read_input_file("inputs/"+curMedium)
        T = solve(G.copy())
        assert is_valid_network(G, T)
        write_output_file(T, 'outputs/' + curMedium[0:-3] + '.out')
        print("done with medium")
    small = "small-"
    for i in range(1,304):
        print(i)
        curSmall = small + str(i) +".in"
        G = read_input_file("inputs/"+curSmall)
        T = solve(G.copy())
        assert is_valid_network(G, T)
        write_output_file(T, 'outputs/' + curSmall[0:-3] + '.out')
        print("done with small")
    
