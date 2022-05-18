"""
Dijkstra's search for Vehicle Platooning problem with recharging points, where it is assumed that all but the leading vehicle make a 20% charge safe when being in a platoon
@author: Ivaylo Petrov
Comment: The implementation of the Dijkstra algorithm is inspired by https://cristianbastidas.com/my-blog/en/algorithms/python/tutorial/wiki/2021/05/23/python-dijkstra.html
"""

import copy # Used for copying objects
import heapq    # Used for constructing a priority queue structure
from collections import OrderedDict # Used to remove duplicates from a given list
import time # Used to measure the elapsed time when testing the algorithm

class Dijkstra:
    def __init__(self, n, mat, s_start, s_goal):#, stationss):
        self.s_start, self.s_goal = s_start, s_goal

        self.number_of_nodes = n

        self.matrix = mat

        self.PARENT = dict()
        self.path = []
        self.visited = []

        #Every vehicle has a max charge capacity of 'max_charge' units
        #(100 by default, meaning it can travel 100 km without recharging)
        self.max_charge = 100

    def init(self):
        for i in range(self.number_of_nodes):
            self.PARENT[i] = None
            self.visited.append(False)

    def run(self):
        """
        :return: the total cost for traversing the best found path and the best found path itself
        """

        self.init()

        dist = [float("inf")] * self.number_of_nodes
        dist[self.s_start] = 0

        for _ in range(self.number_of_nodes - 1):
            minix = float("inf")
            u = 0

            for v in range(self.number_of_nodes):
                if self.visited[v] == False and round(dist[v], 2) <= minix:
                    minix = dist[v]
                    u = v

            self.visited[u] = True
            for v in range(self.number_of_nodes):

                if self.visited[v] == False and self.matrix[u][v] != -1:

                    if (round(round(dist[u], 2) + self.matrix[u][v], 2) < round(dist[v], 2)):
                        self.PARENT[v] = u
                        dist[v] = round(round(dist[u], 2) + self.matrix[u][v], 2)

        j = self.s_goal
        s = []
        while self.PARENT[j] != None:
            s.append(j)
            j = self.PARENT[j]
        if round(dist[self.s_goal], 2) == float("inf"):
            return None, []
        s.append(self.s_start)
        self.path = s[::-1]

        return round(dist[self.s_goal], 2), self.path

def nodes_with_stations(object, number_of_nodes, stations):
    """
    Finds all nodes where there is a recharging point at
    :param object: Dijkstra object
    :param number_of_nodes: integer representing the number of nodes in the graph
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :return: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given Dijkstra object (vehicle)
    """
    
    all_stations = [object.s_start, object.s_goal]
    
    for i in range(number_of_nodes):
        if i != object.s_start and i != object.s_goal and stations[i] != 0:
            all_stations.append(i)
    return all_stations

def original_path(paths_to_stations, path):
    """
    Finds the original form of the best found path - restores nodes without recharging points, which also should be part of the final form of the best path
    :param paths_to_stations: dictionary which stores the best path between given two nodes
    :param path: list representing the best found path for the given vehicle (without nodes without recharging points)
    :return: list of integers representing the final form of the best found path
    """

    real_path = []
    for i in range(len(path)-1):
        if (path[i], path[i+1]) in paths_to_stations:
            for j in paths_to_stations[(path[i], path[i+1])]:
                real_path.append(j)
        elif (path[i+1], path[i]) in paths_to_stations:
            for j in paths_to_stations[(path[i+1], path[i])][::-1]:
                real_path.append(j)
    
    return list(OrderedDict.fromkeys(real_path))

def dijkstra_matrix_with_stations_and_GV(dijkstra, station_nodes, corrected_stations, number_of_nodes, matrix_with_reduced_edges):
    """
    Implemets section 2, subsection 2.1, Lemma 1 from http://www.cs.umd.edu/projects/gas/gas-station.pdf - calculating the GV
    
    :param dijkstra: Dijkstra object
    :param station_nodes: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given Dijkstra object (vehicle)
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    :param number_of_nodes: integer showing the number of nodes in the graph structure
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph    
    
    :return matrix_with_petrol_stations: adjacency matrix where there is a connection between any two nodes if and only if there is a recharging station at both of them and their interdistance is not bigger than the maximum charge capacity of the vehicle
    :return gv: the GV dictionary from section 2, subsection 2.1, Lemma 1 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    :return paths_to_stations: dictionary which stores the best path between given two nodes
    :return nodes_traversing_order: list of integers (nodes) representing the order the graph is traversed - a variable used for faster calculations in a later stage
    """

    all_stations = copy.deepcopy(station_nodes)
    nodes_traversing_order = []  # Stores the correct order in which the nodes are traversed

    paths_to_stations = dict()  # Stores the original shortest paths between any two stations
                                # Format: {(start point, end point): [path]}
    
    gv = dict() # The set of values of gas for vertex u (Section 2, Subsection 2.1, Lemma 1, http://www.cs.umd.edu/projects/gas/gas-station.pdf)

    for i in range(number_of_nodes):
        gv[i] = set()
        gv[i].add(0)

    dist = [float("inf")] * number_of_nodes
    dist[dijkstra.s_start] = 0
    visited = set() # Vertex i is always before vertex j (vertices w and u in http://www.cs.umd.edu/projects/gas/gas-station.pdf (Section 2.1), respectively)
    
    # For its implementation matrix_with_petrol_stations uses the idea of BlueRaja - Danny Pflughoeft found at https://stackoverflow.com/questions/22061009/a-shortest-path-with-fuel-constraints
    matrix_with_petrol_stations = [[-1 for i in range(number_of_nodes)] for i in range(number_of_nodes)]

    n = len(all_stations)
    
    while n > 0:
        minix = float("inf")
        i = 0

        for v in all_stations:
            if v not in visited and dist[v] <= minix:
                minix = dist[v]
                i = v

        if i not in visited:
            visited.add(i)
            nodes_traversing_order.append(i)

        for j in all_stations:
            if j == i or j in visited:
                continue
            
            obj = Dijkstra(number_of_nodes, matrix_with_reduced_edges, i, j)
            
            distance, path = obj.run()
            if path == []:
                continue
            distance = round(distance, 2)
            
            if dijkstra.max_charge >= distance:
                paths_to_stations[(i,j)] = path
                matrix_with_petrol_stations[i][j] = round(distance, 2)
                matrix_with_petrol_stations[j][i] = round(distance, 2)

                if (round(round(dist[i], 2) + round(distance, 2), 2) < round(dist[j], 2)):
                    dist[j] = round(round(dist[i], 2) + round(distance, 2), 2)

                if corrected_stations[i] < corrected_stations[j]:

                    gv[j].add(round(dijkstra.max_charge - distance, 2))

        n -= 1

    return matrix_with_petrol_stations, gv, paths_to_stations, nodes_traversing_order


def dijkstra_graph_H(dijkstra, gv, matrix_with_petrol_stations, all_stations, corrected_stations, nodes_traversing_order, number_of_nodes, matrix_with_reduced_edges):
    """
    Implemets section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    
    :param dijkstra: Dijkstra object
    :param gv: the GV dictionary from section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    :param matrix_with_petrol_stations: adjacency matrix where there is a connection between any two nodes if and only if there is a recharging station at both of them and their interdistance is not bigger than the maximum charge capacity of the vehicle
    :param all_stations: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given Dijkstra object (vehicle)
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    :param nodes_traversing_order: list of integers (nodes) representing the order the graph is traversed
    :param number_of_nodes: integer showing the number of nodes in the graph structure
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph    
    
    :return new_matrix: adjacency matrix representing graph H from section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    :return nodes: dictionary of the nodes of new_matrix
    :return number_of_new_nodes: total number of nodes used in new_matrix
    """

    nodes = dict()
    value = 0
    i = 0
    while i < number_of_nodes:
        
        for j in gv[i]:
            nodes[(i, j)] = value
            value += 1
        i += 1

    number_of_new_nodes = len(nodes)
    new_matrix = [[-1 for i in range(number_of_new_nodes)] for i in range(number_of_new_nodes)]
    visited = set() # We want to ensure that vertex v comes always after vertex u
    n = len(all_stations)
    m = n

    while n > 0:

        u = nodes_traversing_order[m - n]
        visited.add(u)
    
        for v in all_stations:
            if u == v or v in visited:
                continue
            obj = Dijkstra(number_of_nodes, matrix_with_reduced_edges, u, v)
            distance, path = obj.run()
            if distance == None:
                continue
            dist = round(distance, 2)
        
            if dist > dijkstra.max_charge:
                continue

            if corrected_stations[v] <= corrected_stations[u]:

                for g in gv[u]:
                    if g <= dist:
                        new_matrix[nodes[(u, g)]][nodes[(v, 0)]] = round(round((dist - g), 2) * corrected_stations[u], 2) + matrix_with_petrol_stations[u][v]
                        

            elif corrected_stations[v] > corrected_stations[u]:
                
                for g in gv[u]:
                    new_matrix[nodes[(u, g)]][nodes[(v, round(dijkstra.max_charge - dist, 2))]] = round(round((dijkstra.max_charge - g), 2) * corrected_stations[u], 2) + matrix_with_petrol_stations[u][v]
                    
        n -= 1
    
    return new_matrix, nodes, number_of_new_nodes


def dijkstra_find_best_path(dijkstra, corrected_stations, stations, matrix_with_reduced_edges, covered_edges, number_of_nodes, first_run):
    """
    Main function which combines all the smaller bits and calculates the best path for the given vehicle and the cost for traversing it
    
    :param dijkstra: Dijkstra object
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph    
    :param covered_edges: set which stores which edges have been covered by at least one vehicle
    :param number_of_nodes: integer showing the number of nodes in the graph structure
    :param first_run: boolean

    :return total_cost: integer showing the total cost the given vehicle will experience when for going through the best found path for it
    :return real_path: array of integers (nodes) representing the best path for the given vehicle
    """
    all_stations = nodes_with_stations(dijkstra, number_of_nodes, stations)

    matrix_with_petrol_stations, gv, paths_to_stations, nodes_traversing_order = dijkstra_matrix_with_stations_and_GV(dijkstra, copy.deepcopy(all_stations), corrected_stations, number_of_nodes, matrix_with_reduced_edges)

    new_matrix, nodes, number_of_new_nodes = dijkstra_graph_H(dijkstra, copy.deepcopy(gv), matrix_with_petrol_stations, copy.deepcopy(all_stations), corrected_stations, nodes_traversing_order, number_of_nodes, matrix_with_reduced_edges)

    if (dijkstra.s_goal, 0) not in nodes or (dijkstra.s_start, 0) not in nodes:
        return -1, []

    # Main Dijkstra object, which runs through the final form of the matrix (graph H)
    obj = Dijkstra(number_of_new_nodes, new_matrix, nodes[(dijkstra.s_start, 0)], nodes[(dijkstra.s_goal, 0)])
    
    distance, path = obj.run()
    if path == []:
        return -1, []

    final_path = []
        
    for i in obj.path:
        final_path.append(next((key[0] for (key, value) in nodes.items() if value == i), None))

    real_path = original_path(paths_to_stations, final_path)

    final_path = final_path[1:-1]

    total_cost = distance # The total cost is the sum of the cost for traversing the selected path and the total cost for recharging the vehicle
    
    for i in range(len(real_path)-1):
        # if first_run is True, then the covered edge's weight is not reduced because we run this function just to calculate the best path of the current vehicle without considering platooning
        # We check if the current edge has already been traversed in the same direction only because the vehicle will move this exact direction.
        if first_run == False:
            if (real_path[i], real_path[i+1]) not in covered_edges:
                covered_edges.add((real_path[i], real_path[i+1]))
                matrix_with_reduced_edges[real_path[i]][real_path[i+1]] *= 0.8
                matrix_with_reduced_edges[real_path[i]][real_path[i+1]] = round(matrix_with_reduced_edges[real_path[i]][real_path[i+1]], 2)
    
    return total_cost, real_path

def main(start_end_points, tank_capacities, number_of_vehicles, stations, number_of_nodes, matrix_with_reduced_edges, covered_edges):
    # Main function which returns the time needed the algorithm to run and the total safe made by the algorithm

    start_end_points = copy.deepcopy(start_end_points)
    tank_capacities = copy.deepcopy(tank_capacities)
    best_paths_without_platooning = []  # stores the order in which the vehicles will be traversed - from the one with the longest best path to the one with the shortest best path

    max_priority_queue = [] # used for sorting the vehicles by the length of their best paths
    costs_without_platooning = dict()   # stores the cost of every vehicles going through its best path without considering platooning

    start = time.time() # start measuring the time

    for i in range(number_of_vehicles):
        corrected_stations = copy.deepcopy(stations)
        corrected_stations[start_end_points[i][0]] = 0
        corrected_stations[start_end_points[i][1]] = 0
        dijkstra_object = Dijkstra(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        dijkstra_object.max_charge = tank_capacities[i]
        total_cost, real_path = dijkstra_find_best_path(dijkstra_object, corrected_stations, stations, matrix_with_reduced_edges, covered_edges, number_of_nodes, True)
        if real_path == []:    # there is no way how the given vehicle can go from its start point to its final destination
            continue
        heapq.heappush(max_priority_queue, (len(real_path)*(-1), i))
        costs_without_platooning[i] = total_cost

    for i in range(len(max_priority_queue)):
        best_paths_without_platooning.append(heapq.heappop(max_priority_queue)[1])

    total_safe = 0
    for i in best_paths_without_platooning:
        corrected_stations = copy.deepcopy(stations)
        corrected_stations[start_end_points[i][0]] = 0
        corrected_stations[start_end_points[i][1]] = 0
        dijkstra_object = Dijkstra(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        dijkstra_object.max_charge = tank_capacities[i]
        cost_with_platooning = dijkstra_find_best_path(dijkstra_object, corrected_stations, stations, matrix_with_reduced_edges, covered_edges, number_of_nodes, False)[0]
        total_safe += (costs_without_platooning[i] - cost_with_platooning)

    end = time.time()   # stop measuring the time

    return round(end - start, 3), round(total_safe, 2)