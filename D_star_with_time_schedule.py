"""
D* search with printed best found schedule for every vehicle in Vehicle Platooning problem with recharging points
@author: Ivaylo Petrov
Comment: The DStar class is a modified version of the huiming zhou's version, which can be found at 
         https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_2D/D_star.py
"""

import math # Used for mathematical functions such as ceil
import random   # Used for generating random numbers
import copy # Used for copying objects
import networkx as nx   # Used for generating random graph structures
import heapq    # Used for constructing a priority queue structure
from collections import OrderedDict # Used to remove duplicates from a given list

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

        self.speed = 80 # Represents the average speed of the vehicle - 80 km/h is the default value
        self.total_travel_time = 0  # The total time needed to travel through the cheapest path

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


def nodes_with_stations(object):
    """
    Finds all nodes where there is a recharging point at
    :param object: DStar object
    :return: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given DStar object (vehicle)
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

def dstar_matrix_with_stations_and_GV(matrix_with_reduced_edges, dstar, station_nodes, corrected_stations):
    """
    Implemets section 2, subsection 2.1, Lemma 1 from http://www.cs.umd.edu/projects/gas/gas-station.pdf - calculating the GV
    
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    :param dstar: DStar object
    :param station_nodes: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given DStar object (vehicle)
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    
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

def dstar_graph_H(matrix_with_reduced_edges, dstar, gv, matrix_with_petrol_stations, all_stations, corrected_stations, nodes_traversing_order):
    """
    Implemets section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf - calculating the GV
    
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph    
    :param dstar: DStar object
    :param gv: the GV dictionary from section 2, subsection 2.1, Theorem 2 from http://www.cs.umd.edu/projects/gas/gas-station.pdf
    :param matrix_with_petrol_stations: adjacency matrix where there is a connection between any two nodes if and only if there is a recharging station at both of them and their interdistance is not bigger than the maximum charge capacity of the vehicle
    :param all_stations: list of integers representing every node with a recharging point at plus the initial and goal nodes for the given DStar object (vehicle)
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    :param nodes_traversing_order: list of integers (nodes) representing the order the graph is traversed
    
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

def from_minutes_to_hours(minutes):
    """
    Transforms given minutes to the format hh:mm
    :param minutes: integer
    :return string format (hh:mm) representing the elapsed time since 00:00
    """
    
    days = 0
    while minutes >= 1440: # One full day (24h) has 1440 minutes
        days += 1
        minutes -= 1440

    hours = int(minutes/60)
    hours = f"{hours:02d}"
    minutes %= 60
    minutes = int(minutes)
    minutes = f"{minutes:02d}"
    if days == 0:
        return hours + ':' + minutes
    elif days == 1:
        return hours + ':' + minutes + " on the next day"
    else:
        return hours + ':' + minutes + " after " + str(days) + " days"

def dstar_print_path_information(original_matrix, truck_number, dstar_obj, stations_fueling, time_charge_at_station, final_path, real_path, nodes_departure_times, to_print):
    """
    Prints the best found schedule for the given truck in a nice format
    
    :param original_matrix: adjacency matrix representing the original form of the input graph structure
    :param truck_number: integer value representing the "id" of the current DStar object (current vehicle)
    :param dstar_object: DStar object
    :param stations_fueling: list of integers showing how many electric charge units the vehicle should charge at given recharging point - stations_fueling[index] shows the amount recharged at final_path[index]
    :param time_charge_at_station: list of integers representing how much time is needed to recharge 1 unit of charge at a given point - time_charge_at_station[index] shows the time to recharge 1 unit of charge at final_path[index]
    :param final_path: list of integers representing the recharging points where the vehicle stops to recharge at
    :param real_path: array of integers (nodes) representing the best path for the given vehicle
    :param nodes_departure_times: dictionary in the format {(start, end): (x, y)}, where all vehicles going through edge start - end will go from node start at time x and will arrive at node end at time y
    :param to_print: boolean variable which if is True, the gathered information is printed
    """
    
    if to_print:
        print("Truck {}:".format(truck_number))
        print("The vehicle starts its journey from node {} at {}".format(real_path[0], from_minutes_to_hours(nodes_departure_times[(real_path[0], real_path[1])][0])))
        print()

    length_final_path = len(final_path)

    current_node = real_path[0]
    is_refueld = False
    ind = 0 # follows the recharging stations and the corresponding recharging amounts
    i = 1

    while i < len(real_path)-1:
        current_node = real_path[i]

        if is_refueld == False:
            time_between_last_two_nodes = math.ceil((original_matrix[real_path[i]][real_path[i-1]] / float(dstar_obj.speed)) * 60)

            if to_print:
                print("The vehicle drives {} minutes to node {} and arrives there at {}".format(time_between_last_two_nodes, current_node, from_minutes_to_hours(nodes_departure_times[(real_path[i-1], real_path[i])][1])))

        if is_refueld:
            i -= 1
            is_refueld = False

        if ind < length_final_path and current_node == final_path[ind] and is_refueld == False:
            if stations_fueling[ind] != 0:
                if to_print:
                    print("The vehicle recharges {} amounts of energy for {} units of time (minutes) per 1 amount of energy. Consequently, the vehicle spends {} units of time (minutes) for recharging at this station.".format(stations_fueling[ind], time_charge_at_station[ind], round(stations_fueling[ind] * time_charge_at_station[ind],2)))
                
                is_refueld = True
            ind += 1
            
        if is_refueld:
            is_refueld = False

        i += 1
        if is_refueld == False or i==len(real_path)-1:
            if to_print:
                print()

    time_between_last_two_nodes = math.ceil((original_matrix[real_path[i]][real_path[i-1]] / float(dstar_obj.speed)) * 60)

    if to_print:
        print("The vehicle drives {} minutes to node {} (its final destination) and arrives there at {}".format(time_between_last_two_nodes, real_path[i], from_minutes_to_hours(nodes_departure_times[(real_path[i-1], real_path[i])][1])))
        print()


def dstar_find_best_path(original_matrix, matrix_with_reduced_edges, truck_number, dstar, corrected_stations, first_run, nodes_departure_times, data_to_be_printed, covered_edges):
    """
    Main function which combines all the smaller bits and calculates the best path for the given vehicle and the cost for traversing it
    
    :param original_matrix: adjacency matrix representing the original form of the input graph structure
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph    
    :param truck_number: integer value representing the "id" of the current DStar object (current vehicle)
    :param dstar: DStar object
    :param corrected_stations: list of integers where corrected_stations[index] shows the time (in minutes) needed for recharging the vehicle by 1 amount of energy (enough for travelling 1 km) at station index - recharging cost at start and end node is 0
    :param first_run: boolean
    :param nodes_departure_times: dictionary in the format {(start, end): (x, y)}, where all vehicles going through edge start - end will go from node start at time x and will arrive at node end at time y
    :param data_to_be_printed: dictionary structure which stores all the information that is needed to be printed for every vehicle
    :param covered_edges: set which stores which edges have been covered by at least one vehicle
    
    :return total_cost: integer showing the total cost the given vehicle will experience when for going through the best found path for it
    :return real_path: array of integers (nodes) representing the best path for the given vehicle
    :return data_to_be_printed: dictionary structure which stores all the information that is needed to be printed for every vehicle
    :return nodes_departure_times: dictionary in the format {(start, end): (x, y)}, where all vehicles going through edge start - end will go from node start at time x and will arrive at node end at time y
    :return covered_edges: set which stores which edges have been covered by at least one vehicle
    """

    all_stations = nodes_with_stations(dstar)

    matrix_with_petrol_stations, gv, paths_to_stations, nodes_traversing_order = dstar_matrix_with_stations_and_GV(matrix_with_reduced_edges, dstar, copy.deepcopy(all_stations), corrected_stations)

    new_matrix, nodes, number_of_new_nodes = dstar_graph_H(matrix_with_reduced_edges, dstar, gv, matrix_with_petrol_stations, copy.deepcopy(all_stations), corrected_stations, nodes_traversing_order)

    if (dstar.s_goal, 0) not in nodes or (dstar.s_start, 0) not in nodes:
        return -1, [], {}, {}, []

    # Main DStar object, which runs through the final form of the matrix (graph H)
    obj = DStar(number_of_new_nodes, new_matrix, nodes[(dstar.s_goal, 0)], nodes[(dstar.s_start, 0)])

    if obj.run() == []:
        return -1, [], {}, {}, []

    obj.path = obj.path[::-1]

    final_path = []
        
    for i in obj.path:
        final_path.append(next((key[0] for (key, value) in nodes.items() if value == i), None))

    real_path = original_path(paths_to_stations, final_path)

    stations_fueling = []
    time_charge_at_station = []
    
    for i in range(1, len(obj.path)-1):
        stations_fueling.append(round(round(new_matrix[obj.path[i]][obj.path[i+1]] - matrix_with_petrol_stations[final_path[i]][final_path[i+1]], 2) / stations[final_path[i]], 2))
        time_charge_at_station.append(round(stations[final_path[i]], 2))

    final_path = final_path[1:-1]

    total_cost = obj.h[nodes[(dstar.s_goal, 0)]]  # The total cost is the sum of the cost for traversing the selected path and the total cost for recharging the vehicle

    dstar.total_travel_time = 0

    time_between_first_two_nodes = math.ceil((original_matrix[real_path[0]][real_path[1]] / float(dstar.speed)) * 60)

    for i in range(len(real_path)-1):

        # if first_run is True, then the covered edge's weight is not reduced because we run this function just to calculate the best path of the current vehicle without considering platooning
        # We check if the current edge has already been traversed in the same direction only because the vehicle will move this exact direction.
        if first_run == False:
        
            if (real_path[i], real_path[i+1]) not in covered_edges:

                covered_edges.add((real_path[i], real_path[i+1]))
                matrix_with_reduced_edges[real_path[i+1]][real_path[i]] *= 0.8
                matrix_with_reduced_edges[real_path[i+1]][real_path[i]] = round(matrix_with_reduced_edges[real_path[i+1]][real_path[i]], 2)

                time_between_last_two_nodes = math.ceil((original_matrix[real_path[i]][real_path[i+1]] / float(dstar.speed)) * 60)

                if i == 0:

                    if (real_path[0], real_path[1]) in nodes_departure_times:
                        start = max(0, nodes_departure_times[(real_path[i], real_path[i+1])][0])
                        end = max(time_between_first_two_nodes + start, nodes_departure_times[(real_path[i], real_path[i+1])][1])
                        nodes_departure_times[(real_path[i], real_path[i+1])] = (start, end)
                        dstar.total_travel_time = start
                        dstar.total_travel_time += time_between_last_two_nodes

                    else:
                        dstar.total_travel_time += time_between_last_two_nodes
                        nodes_departure_times[(real_path[i], real_path[i+1])] = (0, time_between_first_two_nodes)

                else:

                    if (real_path[i], real_path[i+1]) in nodes_departure_times:

                        if (real_path[i-1], real_path[i]) in nodes_departure_times and dstar.total_travel_time < nodes_departure_times[(real_path[i-1], real_path[i])][0]:
                            dstar.total_travel_time = nodes_departure_times[(real_path[i-1], real_path[i])][1]
                        
                        dstar.total_travel_time += time_between_last_two_nodes
                        current_departure_time = dstar.total_travel_time

                        if real_path[i] in final_path:
                            ind = final_path.index(real_path[i])
                            current_departure_time += stations_fueling[ind] * time_charge_at_station[ind]
                            dstar.total_travel_time = current_departure_time

                            if int(nodes_departure_times[(real_path[i], real_path[i+1])][1]) < int(dstar.total_travel_time):
                                nodes_departure_times[(real_path[i], real_path[i+1])] = (dstar.total_travel_time - time_between_last_two_nodes, dstar.total_travel_time)
                                return None, [], {}, nodes_departure_times, []

                        else:
                            if nodes_departure_times[(real_path[i], real_path[i+1])][1] < dstar.total_travel_time:
                                nodes_departure_times[(real_path[i], real_path[i+1])] = (nodes_departure_times[(real_path[i-1], real_path[i])][1], current_departure_time)
                                return None, [], {}, nodes_departure_times, []

                    else:
                        dstar.total_travel_time += time_between_last_two_nodes
                        current_departure_time = dstar.total_travel_time

                        if real_path[i] in final_path:
                            ind = final_path.index(real_path[i])
                            current_departure_time += stations_fueling[ind] * time_charge_at_station[ind]
                            dstar.total_travel_time = current_departure_time

                            nodes_departure_times[(real_path[i], real_path[i+1])] = (dstar.total_travel_time - time_between_last_two_nodes, dstar.total_travel_time)

                        else:
                            nodes_departure_times[(real_path[i], real_path[i+1])] = (nodes_departure_times[(real_path[i-1], real_path[i])][1], dstar.total_travel_time)# + time_between_first_two_nodes)


            else:
                time_between_last_two_nodes = math.ceil((original_matrix[real_path[i]][real_path[i+1]] / float(dstar.speed)) * 60)

                if i == 0:
                    start = max(0, nodes_departure_times[(real_path[i], real_path[i+1])][0])
                    end = max(time_between_first_two_nodes + start, nodes_departure_times[(real_path[i], real_path[i+1])][1])
                    nodes_departure_times[(real_path[i], real_path[i+1])] = (start, end)
                    dstar.total_travel_time = start
                    dstar.total_travel_time += time_between_last_two_nodes
                    
                else:
                    
                    if (real_path[i-1], real_path[i]) in nodes_departure_times and dstar.total_travel_time < nodes_departure_times[(real_path[i-1], real_path[i])][0]:
                        dstar.total_travel_time = nodes_departure_times[(real_path[i-1], real_path[i])][1]
                    
                    dstar.total_travel_time += time_between_last_two_nodes
                    current_departure_time = dstar.total_travel_time

                    if real_path[i] in final_path:
                        ind = final_path.index(real_path[i])
                        current_departure_time += stations_fueling[ind] * time_charge_at_station[ind]
                        dstar.total_travel_time = current_departure_time

                        if int(nodes_departure_times[(real_path[i], real_path[i+1])][1]) < int(dstar.total_travel_time):
                            nodes_departure_times[(real_path[i], real_path[i+1])] = (dstar.total_travel_time - time_between_last_two_nodes, dstar.total_travel_time)
                            return None, [], {}, nodes_departure_times, []

                    else:
                        if nodes_departure_times[(real_path[i], real_path[i+1])][1] < dstar.total_travel_time:
                            nodes_departure_times[(real_path[i], real_path[i+1])] = (nodes_departure_times[(real_path[i-1], real_path[i])][1], current_departure_time)
                            return None, [], {}, nodes_departure_times, []


    data_to_be_printed[truck_number] = [dstar, stations_fueling, time_charge_at_station, final_path, real_path]

    return total_cost, real_path, data_to_be_printed, nodes_departure_times, covered_edges

def main(original_matrix, number_of_vehicles, start_end_points, tank_capacities, stations, matrix_with_reduced_edges, nodes_departure_times, data_to_be_printed, covered_edges):
    # Main function
    
    start_end_points = copy.deepcopy(start_end_points)
    tank_capacities = copy.deepcopy(tank_capacities)
    best_paths_without_platooning = []

    max_priority_queue = []
    costs_without_platooning = dict()

    for i in range(number_of_vehicles):
        corrected_stations = copy.deepcopy(stations)
        corrected_stations[start_end_points[i][0]] = 0
        corrected_stations[start_end_points[i][1]] = 0
        dstar_object = DStar(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        dstar_object.max_charge = tank_capacities[i]
        total_cost, real_path, _, _, _ = dstar_find_best_path(copy.deepcopy(original_matrix), matrix_with_reduced_edges, i, dstar_object, corrected_stations, True, nodes_departure_times, dict(), covered_edges)
        if real_path == []:
            continue
        heapq.heappush(max_priority_queue, (len(real_path)*(-1), i))
        costs_without_platooning[i] = total_cost

    for i in range(len(max_priority_queue)):
        best_paths_without_platooning.append(heapq.heappop(max_priority_queue)[1])

    data_to_be_printed = dict()

    final_paths = [[] for i in range(number_of_vehicles)]
    i = 0
    while i < len(best_paths_without_platooning):
        corrected_stations = copy.deepcopy(stations)
        corrected_stations[start_end_points[best_paths_without_platooning[i]][0]] = 0
        corrected_stations[start_end_points[best_paths_without_platooning[i]][1]] = 0
        dstar_object = DStar(number_of_nodes, matrix_with_reduced_edges, start_end_points[best_paths_without_platooning[i]][0], start_end_points[best_paths_without_platooning[i]][1])
        dstar_object.max_charge = tank_capacities[best_paths_without_platooning[i]]
        cost_with_platooning, real_path, data_to_be_printed, nodes_departure_times, covered_edges = dstar_find_best_path(copy.deepcopy(original_matrix), matrix_with_reduced_edges, best_paths_without_platooning[i], dstar_object, corrected_stations, False, nodes_departure_times, data_to_be_printed, covered_edges)
        
        if cost_with_platooning == None:
            i = 0
            matrix_with_reduced_edges = copy.deepcopy(original_matrix)
            data_to_be_printed = dict()
            covered_edges = set()
            continue

        if cost_with_platooning == -1:
            i += 1
            continue
        
        final_paths[i] = real_path
        i += 1
        
    if best_paths_without_platooning != []:
        for i in best_paths_without_platooning:
            dstar_print_path_information(copy.deepcopy(original_matrix), i, data_to_be_printed[i][0], data_to_be_printed[i][1], data_to_be_printed[i][2], data_to_be_printed[i][3], data_to_be_printed[i][4], nodes_departure_times, True)

if __name__ == '__main__':

    """# Below is a sample input data
    number_of_nodes = 8
    original_matrix = [[-1 for i in range(number_of_nodes)] for i in range(number_of_nodes)]

    original_matrix[7][2] = 110
    original_matrix[2][7] = 110
    original_matrix[0][1] = 50
    original_matrix[1][0] = 50
    original_matrix[1][2] = 40
    original_matrix[2][1] = 40
    original_matrix[2][4] = 100
    original_matrix[4][2] = 100
    original_matrix[1][3] = 60
    original_matrix[3][1] = 60
    original_matrix[3][4] = 80
    original_matrix[4][3] = 80
    original_matrix[2][5] = 10
    original_matrix[5][2] = 10
    original_matrix[5][6] = 10
    original_matrix[6][5] = 10
    original_matrix[4][6] = 80
    original_matrix[6][4] = 80

    stations = [15,0,20,10,0,10,10,100]

    matrix_with_reduced_edges = copy.deepcopy(original_matrix)

    covered_edges = set()

    nodes_departure_times = dict()

    start_end_points = [(1, 4), (0, 4), (7,6), (4,1), (3,0)]
    tank_capacities = []
    number_of_vehicles = 5

    for _ in range(number_of_vehicles):
        tank_capacities.append(110)
        
    main(original_matrix, number_of_vehicles, start_end_points, tank_capacities, stations, matrix_with_reduced_edges, nodes_departure_times, dict(), covered_edges)"""

    # Below is a randomly generated input data

    number_of_nodes = 5
    number_of_vehicles = 5

    graph = nx.erdos_renyi_graph(number_of_nodes,0.5)

    for (u,v,w) in graph.edges(data=True):
        w['weight'] = float(random.randint(0, 100))
    original_matrix = nx.to_numpy_array(graph, dtype=float).tolist()
    for i in range(number_of_nodes):
        for j in range(number_of_nodes):
            if original_matrix[i][j] == 0:
                original_matrix[i][j] = -1

    stations = [random.randint(0, 20) for _ in range(number_of_nodes)]  # Represents if there is a recharging station at a given node - if 0, then
                                                # there is no station, otherwise the number shows how much time (cost) it takes
                                                # to charge the vehicle with enough energy to travel 1 unit of distance (km)

    start_end_points = []
    tank_capacities = []

    for _ in range(number_of_vehicles):
        start, end = 0, 0
        while start == end:
            start, end = random.randint(0, number_of_nodes - 1), random.randint(0, number_of_nodes - 1)
        start_end_points.append((start, end))
        tank_capacities.append(random.randint(80, 100))

    matrix_with_reduced_edges = copy.deepcopy(original_matrix)

    covered_edges = set()

    nodes_departure_times = dict()

    main(original_matrix, number_of_vehicles, start_end_points, tank_capacities, stations, matrix_with_reduced_edges, nodes_departure_times, dict(), covered_edges)