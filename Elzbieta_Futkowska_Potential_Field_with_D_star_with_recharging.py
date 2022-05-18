"""
D* search implementing Elzbieta Futkowska's potential field method for heuristically solving the Vehicle Platooning problem with recharging points, where it is assumed that all vehicles in a platoon make 20% charge safe (including the leading vehicle)
@author: Ivaylo Petrov
Comment: The DStar class is a modified version of the huiming zhou's version, which can be found at 
         https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_2D/D_star.py
"""

import copy # Used for copying objects
from collections import OrderedDict # Used to remove duplicates from a given list
import time # Used to measure the elapsed time when testing the algorithm

# D* search implementation
class DStar:
    def __init__(self, n, mat, s_start, s_goal):
        self.s_start, self.s_goal = s_start, s_goal

        self.number_of_nodes = n
        self.matrix = mat
        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()
        self.k = dict()
        self.path = []
        self.visited = set()

        #Every vehicle has a max charge capacity of 'max_charge' units
        #(100 by default, meaning it can travel 100 km without recharging)
        self.max_charge = 100

    def init(self):

        for i in range(self.number_of_nodes):
            self.t[i] = 'NEW'
            self.k[i] = 0.0
            self.h[i] = float("inf")
            self.PARENT[i] = None


        self.h[self.s_goal] = 0.0

    def run(self):
        self.init()
        self.insert(self.s_goal, 0)

        while True:
            
            k_min = self.process_state()
            if self.t[self.s_start] == 'CLOSED':
                break

            if k_min == -1 and self.t[self.s_start] != 'CLOSED':    # there is no connection between self.s_start and self.s_goal
                return []
            
        self.path = self.extract_path(self.s_start, self.s_goal)
        return self.path

    def extract_path(self, s_start, s_end):
        path = [s_start]
        s = s_start
        while True:
            s = self.PARENT[s]
            path.append(s)
            if s == s_end:
                return path

    def process_state(self):
        s = self.min_state()  # get node in OPEN set with min k value
        self.visited.add(s)

        if s is None:
            return -1  # OPEN set is empty

        k_old = round(self.get_k_min(), 2)  # record the min k value of this iteration (min path cost)
        self.delete(s)  # move state s from OPEN set to CLOSED set

        # k_min < h[s] --> s: RAISE state (increased cost)
        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                if s_n not in self.visited:
                    if round(self.h[s_n], 2) <= round(k_old, 2) and round(self.h[s], 2) > round(round(self.h[s_n], 2) + self.cost(s_n, s), 2):

                        # update h_value and choose parent
                        self.PARENT[s] = s_n
                        self.h[s] = round(round(self.h[s_n],2) + round(self.cost(s_n, s), 2), 2)

        # s: k_min >= h[s] -- > s: LOWER state (cost reductions)
        if k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                if s_n not in self.visited:
                    if self.t[s_n] == 'NEW' or \
                            (self.PARENT[s_n] == s and round(self.h[s_n], 2) != round(round(self.h[s], 2) + round(self.cost(s, s_n), 2), 2)) or \
                            (self.PARENT[s_n] != s and round(self.h[s_n], 2) > round(round(self.h[s], 2) + round(self.cost(s, s_n), 2), 2)):

                        # Condition:
                        # 1) t[s_n] == 'NEW': not visited
                        # 2) s_n's parent: cost reduction
                        # 3) s_n find a better parent
                        self.PARENT[s_n] = s
                        self.insert(s_n, round(round(self.h[s], 2) + round(self.cost(s, s_n), 2), 2))

        else:
            for s_n in self.get_neighbor(s):
                if s_n not in self.visited:
                    if self.t[s_n] == 'NEW' or \
                            (self.PARENT[s_n] == s and round(self.h[s_n], 2) != round(round(self.h[s], 2) + round(self.cost(s, s_n), 2), 2)):

                        # Condition:
                        # 1) t[s_n] == 'NEW': not visited
                        # 2) s_n's parent: cost reduction
                        self.PARENT[s_n] = s
                        self.insert(s_n, round(round(self.h[s], 2) + round(self.cost(s, s_n), 2), 2))

                    else:
                        if self.PARENT[s_n] != None and self.PARENT[s_n] != s and \
                                round(self.h[s_n], 2) > round(round(self.h[s], 2) + round(self.cost(s, s_n), 2), 2):

                            # Condition: LOWER happened in OPEN set (s), s should be explored again
                            self.insert(s, round(self.h[s],2))
                        else:
                            if self.PARENT[s_n] != None and self.PARENT[s_n] != s and round(self.h[s], 2) > round(round(self.h[s_n], 2) + round(self.cost(s_n, s), 2), 2) and \
                                    self.t[s_n] == 'CLOSED' and round(self.h[s_n], 2) > k_old:

                                # Condition: LOWER happened in CLOSED set (s_n), s_n should be explored again
                                self.insert(s_n, round(self.h[s_n],2))
        
        return round(self.get_k_min(), 2)

    def min_state(self):
        """
        choose the node with the minimum k value in OPEN set.
        :return: state
        """

        if not self.OPEN:
            return None

        return min(self.OPEN, key=lambda x: self.k[x])

    def get_k_min(self):
        """
        calc the min k value for nodes in OPEN set.
        :return: k value
        """

        if not self.OPEN:
            return -1

        return min([self.k[x] for x in self.OPEN])

    def insert(self, s, h_new):
        """
        insert node into OPEN set.
        :param s: node
        :param h_new: new or better cost to come value
        """

        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)

        self.h[s] = round(h_new,2)
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        """
        delete: move state s from OPEN set to CLOSED set.
        :param s: state should be deleted
        """

        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'

        self.OPEN.remove(s)

    def get_neighbor(self, s):
        """
        find all the nodes which node s has a connection with
        :param s: node s
        :return: set of all nodes which node s has a connection with
        """
        neigbour_list = set()

        for u in range(self.number_of_nodes):
            if self.matrix[s][u]!=-1:
                neigbour_list.add(u)

        return neigbour_list

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        """
        return round(self.matrix[s_start][s_goal], 2)


def nodes_with_stations(object, stations, number_of_nodes):
    """
    Finds all nodes where there is a recharging point at
    :param object: DStar object
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :param number_of_nodes: integer representing the number of nodes in the graph
    :return: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given DStar object (vehicle)
    """

    all_stations = [object.s_start, object.s_goal]
    
    for i in range(number_of_nodes):
        if i != object.s_start and i != object.s_goal and stations[i] != 0:
            all_stations.append(i)
        i+=1
        
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

def dstar_matrix_with_stations_and_GV(dstar, station_nodes, corrected_stations, number_of_nodes, matrix_with_reduced_edges):
    """
    Implemets section 2, subsection 2.1, Lemma 1 from http://www.cs.umd.edu/projects/gas/gas-station.pdf - calculating the GV
    
    :param dstar: DStar object
    :param station_nodes: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given DStar object (vehicle)
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
    dist[dstar.s_goal] = 0
    visited = set() # Vertex i is always before vertex j (vertices w and u in http://www.cs.umd.edu/projects/gas/gas-station.pdf (Section 2, Subsection 2.1, Lemma 1), respectively)
    
    # For its implementation matrix_with_petrol_stations uses the idea of BlueRaja - Danny Pflughoeft found at https://stackoverflow.com/questions/22061009/a-shortest-path-with-fuel-constraints
    matrix_with_petrol_stations = [[-1 for i in range(number_of_nodes)] for i in range(number_of_nodes)]

    n = len(all_stations)
    
    while n > 0:
        minix = float("inf")
        j = 0
        
        for v in all_stations:
            if v not in visited and dist[v] <= minix:
                minix = dist[v]
                j = v
                
        if j not in visited:
            visited.add(j)
            nodes_traversing_order.append(j)

        for i in all_stations:
            if j == i or i in visited:
                continue
            
            obj = DStar(number_of_nodes, matrix_with_reduced_edges, i, j)
            
            path = obj.run()
            if path == []:
                continue
            obj.h[i] = round(obj.h[i], 2)
            
            if dstar.max_charge >= obj.h[i]:
                paths_to_stations[(i,j)] = path
                matrix_with_petrol_stations[i][j] = round(obj.h[i], 2)
                matrix_with_petrol_stations[j][i] = round(obj.h[i], 2)

                if (dist[j] + obj.h[i]) < dist[i]:
                    dist[i] = dist[j] + obj.h[i]

                if corrected_stations[i] < corrected_stations[j]:
                    gv[j].add(round(dstar.max_charge - round(obj.h[i], 2), 2))

        n -= 1

    return matrix_with_petrol_stations, gv, paths_to_stations, nodes_traversing_order

def dstar_graph_H(dstar, gv, matrix_with_petrol_stations, all_stations, corrected_stations, nodes_traversing_order, number_of_nodes, matrix_with_reduced_edges):
    """
    Implemets section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    
    :param dstar: DStar object
    :param gv: the GV dictionary from section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    :param matrix_with_petrol_stations: adjacency matrix where there is a connection between any two nodes if and only if there is a recharging station at both of them and their interdistance is not bigger than the maximum charge capacity of the vehicle
    :param all_stations: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given DStar object (vehicle)
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
        v = nodes_traversing_order[m - n]
        visited.add(v)
        for u in all_stations:
            if u == v or u in visited:
                continue
            obj = DStar(number_of_nodes, matrix_with_reduced_edges, u, v)
            obj.run()
            if obj.path == []:
                continue
            dist = round(obj.h[u], 2)
        
            if dist > dstar.max_charge:
                continue

            if corrected_stations[v] <= corrected_stations[u]:

                for g in gv[u]:

                    if g <= dist:
                        new_matrix[nodes[(u, g)]][nodes[(v, 0)]] = round(round((dist - g), 2) * corrected_stations[u], 2) + matrix_with_petrol_stations[u][v]
                        
                
            elif corrected_stations[v] > corrected_stations[u]:
                for g in gv[u]:
                    
                    new_matrix[nodes[(u, g)]][nodes[(v, round(dstar.max_charge - dist, 2))]] = round(round((dstar.max_charge - g), 2) * corrected_stations[u], 2) + matrix_with_petrol_stations[u][v]
                
        n -= 1
        
    return new_matrix, nodes, number_of_new_nodes

def dstar_find_best_path(dstar, corrected_stations, matrix_with_reduced_edges, stations, number_of_nodes):
    """
    Main function which combines all the smaller bits and calculates the best path for the given vehicle and the cost for traversing it
    
    :param dstar: DStar object
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :param number_of_nodes: integer showing the number of nodes in the graph structure

    :return total_cost: integer showing the total cost the given vehicle will experience when for going through the best found path for it
    :return real_path: array of integers (nodes) representing the best path for the given vehicle
    :return final_path: list of integers representing the recharging points where the vehicle stops to recharge at
    :return stations_fueling: list of integers showing how many electric charge units the vehicle should charge at given recharging point - stations_fueling[index] shows the amount recharged at final_path[index]
    """

    all_stations = nodes_with_stations(dstar, stations, number_of_nodes)

    matrix_with_petrol_stations, gv, paths_to_stations, nodes_traversing_order = dstar_matrix_with_stations_and_GV(dstar, copy.deepcopy(all_stations), corrected_stations, number_of_nodes+1, matrix_with_reduced_edges)
    
    new_matrix, nodes, number_of_new_nodes = dstar_graph_H(dstar, gv, matrix_with_petrol_stations, copy.deepcopy(all_stations), corrected_stations, nodes_traversing_order, number_of_nodes+1, matrix_with_reduced_edges)

    if (dstar.s_goal, 0) not in nodes or (dstar.s_start, 0) not in nodes:
        return -1, [], [], []

    # Main DStar object, which runs through the final form of the matrix (graph H)
    obj = DStar(number_of_new_nodes, new_matrix, nodes[(dstar.s_goal, 0)], nodes[(dstar.s_start, 0)])
    
    if obj.run() == [] or obj.h[nodes[(dstar.s_goal, 0)]] == 0:
        return -1, [], [], []

    obj.path = obj.path[::-1]

    final_path = []
        
    for i in obj.path:
        final_path.append(next((key[0] for (key, value) in nodes.items() if value == i), None))

    real_path = original_path(paths_to_stations, final_path)

    stations_fueling = []

    for i in range(1, len(obj.path)-1):
        if stations[final_path[i]]!=0:
            stations_fueling.append(round(round(new_matrix[obj.path[i]][obj.path[i+1]] - matrix_with_petrol_stations[final_path[i]][final_path[i+1]], 2) / stations[final_path[i]], 2))

    final_path = final_path[1:-1]

    total_cost = obj.h[nodes[(dstar.s_goal, 0)]]  # The total cost is the sum of the cost for traversing the selected path and the total cost for recharging the vehicle

    return total_cost, real_path, final_path, stations_fueling

def fixed_path(real_path, stations, max_charge_capacity, matrix_with_reduced_edges):
    """
    having the path a vehicle should travel through, calculate the most optimal cost the vehicle will have - implemets Appendix B from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    :param real_path: an array of integers - the nodes (cities) the vehicle goes through
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :param max_charge_capacity: integer value representing the maximum number of kilometers the vehicle can travel with fully charged battery
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph    
    
    :return cost: the minimum cost to traverse the given path
    """

    distances_between_nodes = []    # Stores the distances between two consecutive nodes
    stations_costs = [] # Stores the recharging costs per unit time at every station which is part of the vehicle's path
    distance_between_last_two_nodes = 0
    ind = 0
    for node in range(len(real_path)-1):

        # Remove the nodes where there is no recharging point at
        if stations[real_path[node+1]] == 0 and node + 1 < len(real_path) - 1:
            distance_between_last_two_nodes += matrix_with_reduced_edges[real_path[node+1]][real_path[node]]
            continue
        
        distances_between_nodes.append(distance_between_last_two_nodes + matrix_with_reduced_edges[real_path[node+1]][real_path[node]])
        stations_costs.append(stations[real_path[ind]])
        distance_between_last_two_nodes = 0
        ind = node + 1

    stations_costs[0] = 0   # The cost to "recharge" at the first node is 0 because we assume that the vehicle is fully recharged at the beginning
    stations_costs.append(0)    # The cost to "recharge" at the last node is 0

    prev = [-1 for _ in range(len(stations_costs))]
    nextt = [-1 for _ in range(len(stations_costs))]
    i, j = 0, 0     # The starting and ending points of the sliding window with size at most the maximum charge capacity of the current vehicle

    while j <= len(distances_between_nodes):
        if prev[j] == -1:
            current_minimum_station_cost = float("inf")
            for k in range(j, i-1, -1):
                if stations_costs[k] < current_minimum_station_cost:
                    current_minimum_station_cost = stations_costs[k]
                    prev[j] = k

        j += 1
        if j > len(distances_between_nodes):
            break
        while sum(distances_between_nodes[i:j]) > max_charge_capacity:
            i += 1

    i = len(distances_between_nodes)
    j = i - 1

    while j >= 0:
        if nextt[j] == -1:
            current_minimum_station_cost = float("inf")
            for k in range(i, j, -1):
                if stations_costs[k] < current_minimum_station_cost:
                    current_minimum_station_cost = stations_costs[k]
                    nextt[j] = k

        j -= 1
        if j < 0:
            break
        while sum(distances_between_nodes[j:i]) > max_charge_capacity:
            i -= 1

    break_points = [i for i in range(len(prev)) if prev[i]==i]

    cost = 0
    i, k = break_points[0], break_points[1]
    ind = 1
    current_amount_of_energy = 0
    while i != k:     # i will be equal to k iff both are at the last break_point
        if sum(distances_between_nodes[i:k]) <= max_charge_capacity:
            cost += ((sum(distances_between_nodes[i:k]) - current_amount_of_energy) * stations_costs[i])
            current_amount_of_energy = 0
            i = k
            if ind < len(break_points) - 1:
                ind += 1
                k = break_points[ind]
        else:
            cost += ((max_charge_capacity - current_amount_of_energy) * stations_costs[i])
            current_amount_of_energy = max_charge_capacity - sum(distances_between_nodes[i:nextt[i]])
            i = nextt[i]
        
        cost = round(cost, 2)

    cost = round(cost, 2)
    cost += sum(distances_between_nodes)
    cost = round(cost, 2)
    return cost


def shortest_paths_for_all_vehicles(initial_amount_of_energy_per_vehicle, number_of_nodes, number_of_vehicles, start_end_points, tank_capacities, stations, matrix_with_reduced_edges):
    """
    Finds the best paths for the vehicles with the given start_end_points on the given matrix_with_reduced_edges
    
    :param initial_amount_of_energy_per_vehicle: list of integers, where initial_amount_of_energy_per_vehicle[index] represents the current amount of energy in vehicle index
    :param number_of_nodes: integer value showing the total number of nodes in the input graph structure
    :param number_of_vehicles: integer value representing the total input number of vehicles
    :param start_end_points: list of tuples where start_end_points[index][0] is the start point for the current vehicle and start_end_points[index][1] is the end point for the vehicle
    :param tank_capacities: list of integers where tank_capacities[index] shows the maximum charge capacity of vehicle index
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    
    :return shortest_paths: list of lists, where shortest_paths[index] shows the best found path for vehicle index
    :return costs_without_platooning: dictionary, where costs_without_platooning[key] shows the total minimum cost for vehicle key to go from its start point to its end point
    :return all_recharging_points: list of lists, where all_recharging_points[index] shows all recharging points on the best path for vehicle index
    :return all_stations_fueling: list of lists, where all_stations_fueling[index] is a list of integers showing how many electric charge units vehicle index should charge at given recharging point - all_stations_fueling[index][ind] shows the amount recharged at all_recharging_points[index][ind]
    :return stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    """
    
    shortest_paths = []
    all_recharging_points = []
    all_stations_fueling = []
    costs_without_platooning = dict()
    
    for i in range(number_of_vehicles):
        old_matrix_with_reduced_edges = copy.deepcopy(matrix_with_reduced_edges)

        for j in range(number_of_nodes):
            matrix_with_reduced_edges[j].append(-1)
        matrix_with_reduced_edges.append([-1] * (number_of_nodes + 1))
        matrix_with_reduced_edges[number_of_nodes][start_end_points[i][0]] = tank_capacities[i] - initial_amount_of_energy_per_vehicle[i]
        matrix_with_reduced_edges[start_end_points[i][0]][number_of_nodes] = tank_capacities[i] - initial_amount_of_energy_per_vehicle[i]
        
        stations.append(0)
        corrected_stations = copy.deepcopy(stations)
        corrected_stations[start_end_points[i][1]] = 0
        dstar_object = DStar(number_of_nodes+1, matrix_with_reduced_edges, number_of_nodes, start_end_points[i][1])
        dstar_object.max_charge = tank_capacities[i]
        total_cost, real_path, recharging_points, stations_fueling = dstar_find_best_path(dstar_object, corrected_stations, matrix_with_reduced_edges, stations, number_of_nodes)
        all_recharging_points.append(recharging_points)
        all_stations_fueling.append(stations_fueling)
        
        if total_cost == -1:
            shortest_paths.append([-1,-1])
            matrix_with_reduced_edges = copy.deepcopy(old_matrix_with_reduced_edges)
            stations = stations[:-1]
            corrected_stations = corrected_stations[:-1]
            continue
        
        shortest_paths.append(real_path[1:])
        costs_without_platooning[i] = total_cost
        matrix_with_reduced_edges = copy.deepcopy(old_matrix_with_reduced_edges)
        stations = stations[:-1]
        corrected_stations = corrected_stations[:-1]

    return shortest_paths, costs_without_platooning, all_recharging_points, all_stations_fueling, stations


def main(number_of_nodes, number_of_vehicles, start_end_points, tank_capacities, stations, original_matrix, matrix_with_reduced_edges, covered_edges):
    """
    Main function which calculates the time needed the algorithm to run and the total safe made by the algorithm

    :param number_of_nodes: integer value showing the total number of nodes in the input graph structure
    :param number_of_vehicles: integer value showing the total input number of vehciles
    :param start_end_points: list of tuples where start_end_points[index][0] is the start point for the current vehicle and start_end_points[index][1] is the end point for the vehicle
    :param tank_capacities: list of integers where tank_capacities[index] shows the maximum charge capacity of vehicle index
    :param stations: list of integers where stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index
    :param original_matrix: adjacency matrix representing the original form of the input graph structure
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    :param covered_edges: set which stores which edges have been covered by at least one vehicle

    :return: total elapsed time from running the algorithm to finishing the calculations
    :return total_safe: float value showing the total safe made by the algorithm considering platooning
    """
    
    start_end_points = copy.deepcopy(start_end_points)
    tank_capacities = copy.deepcopy(tank_capacities)

    start = time.time()

    initial_amount_of_energy_per_vehicle = [tank_capacities[i] for i in range(number_of_vehicles)]  # Shows the start amount of energy in every HDV
    
    # First run of the algorithm to calculate the best paths for all vehicles without considering platooning
    shortest_paths, costs_without_platooning, all_recharging_points, all_stations_fueling, stations = shortest_paths_for_all_vehicles(initial_amount_of_energy_per_vehicle, number_of_nodes, number_of_vehicles, start_end_points, tank_capacities, stations, matrix_with_reduced_edges)
    final_paths = [[] for _ in range(number_of_vehicles)]
    goal_added = [False for _ in range(number_of_vehicles)] # ensures that the final destination for every truck has been added to its final path
    vehicles = [i for i in range(number_of_vehicles)]
    matrix_with_reduced_edges = copy.deepcopy(original_matrix)

    to_be_removed = []  # Stores all the vehicles which we found their best path for and so we remove from the "vehicles" array
    for vehicle in range(number_of_vehicles):
        if shortest_paths[vehicle][0] == -1 and shortest_paths[vehicle][1] == -1:
            to_be_removed.append(vehicle)
            continue
    for i in to_be_removed:
        vehicles.remove(i)

    # Second run of the algorithm to calculate the best paths for all vehicles, considering platooning
    while vehicles:

        to_be_removed = []  # Stores all the vehicles which we found their best path for and so we remove from the "vehicles" array
        
        for t in vehicles:

            final_paths[t].append(shortest_paths[t][0])

            if shortest_paths[t][0] in all_recharging_points[t]:
                ind = all_recharging_points[t].index(shortest_paths[t][0])
                initial_amount_of_energy_per_vehicle[t] += round(all_stations_fueling[t][ind], 2)
            
            initial_amount_of_energy_per_vehicle[t] -= round(matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]],2)

            if (start_end_points[t][0], shortest_paths[t][1]) not in covered_edges:
                matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]] *= 0.8
                matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]] = round(matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]], 2)
                covered_edges[(start_end_points[t][0], shortest_paths[t][1])] = False

            old_matrix_with_reduced_edges = copy.deepcopy(matrix_with_reduced_edges)

            start_end_points[t] = (shortest_paths[t][1], start_end_points[t][1])

            if start_end_points[t][0] == start_end_points[t][1] and goal_added[t] == False:
                to_be_removed.append(t)
                goal_added[t] = True
                final_paths[t].append(start_end_points[t][1])
            shortest_paths, _, all_recharging_points, all_stations_fueling, stations = shortest_paths_for_all_vehicles(initial_amount_of_energy_per_vehicle, number_of_nodes, number_of_vehicles, start_end_points, tank_capacities, stations, matrix_with_reduced_edges)
            matrix_with_reduced_edges = copy.deepcopy(old_matrix_with_reduced_edges)
        
        for i in to_be_removed:
            vehicles.remove(i)

    matrix_with_reduced_edges = copy.deepcopy(original_matrix)
    covered_edges = dict()

    # For every edge, covered by a vehicle, we mark it and when we have a second vehicle going through the same edge, we reduce the edge's weight by 20%
    for final_path in range(len(final_paths)):
        for i in range(len(final_paths[final_path]) - 1):
            if (final_paths[final_path][i+1], final_paths[final_path][i]) not in covered_edges:
                covered_edges[(final_paths[final_path][i+1], final_paths[final_path][i])] = False
            else:
                if covered_edges[(final_paths[final_path][i+1], final_paths[final_path][i])] == False:
                    matrix_with_reduced_edges[final_paths[final_path][i+1]][final_paths[final_path][i]] *= 0.8
                    matrix_with_reduced_edges[final_paths[final_path][i+1]][final_paths[final_path][i]] = round(matrix_with_reduced_edges[final_paths[final_path][i+1]][final_paths[final_path][i]], 2)
                    covered_edges[(final_paths[final_path][i+1], final_paths[final_path][i])] = True

    total_safe = 0
    ind = 0
    # Calculate the final cost for going through the best found paths, using a copy of the original matrix where every edge covered by a vehicle in a platoon has a reduced weight by 20%
    while ind < len(final_paths):
        if final_paths[ind] == []:
            ind += 1
            continue
        current_cost = fixed_path(final_paths[ind], stations, tank_capacities[ind], matrix_with_reduced_edges)
        total_safe += (round(costs_without_platooning[ind], 2) - round(current_cost, 2))
        total_safe = round(total_safe, 2)
            
        ind += 1

    end = time.time()

    return round(end - start, 3), round(total_safe, 2)