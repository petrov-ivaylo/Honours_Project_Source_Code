"""
D* search implementing Elzbieta Futkowska's potential field method for heuristically solving the standard Vehicle Platooning problem - without recharging points
@author: Ivaylo Petrov
Comment: The DStar class is a modified version of the huiming zhou's version, which can be found at 
         https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_2D/D_star.py
"""

import copy # Used for copying objects
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


def shortest_paths_for_all_vehicles(number_of_vehicles, number_of_nodes, start_end_points, matrix_with_reduced_edges):
    """
    Finds the best paths for the vehicles with the given start_end_points on the given matrix_with_reduced_edges

    :param number_of_vehicles: integer value representing the total input number of vehicles
    :param number_of_nodes: integer value showing the total number of nodes in the input graph structure
    :param start_end_points: list of tuples where start_end_points[index][0] is the start point for the current vehicle and start_end_points[index][1] is the end point for the vehicle
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    
    :return shortest_paths: list of lists, where shortest_paths[index] shows the best found path for vehicle index
    :return final_paths: list of lists, where final_paths[index] stores the next node in the best path for vehicle index - look at the Elzbieta Futkowska's report on page 29 - pseudo code for the Potential Field Method inspired solution
    """

    shortest_paths = []
    final_paths = []

    for i in range(number_of_vehicles):
        if start_end_points[i][0] == start_end_points[i][1]:
            shortest_paths.append([-1,-1])
            continue

        dstar_object = DStar(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        real_path = dstar_object.run()
        if real_path == []:
            shortest_paths.append([])
            final_paths.append([])
            continue
        shortest_paths.append(real_path)
        final_paths.append([start_end_points[i][0]])

    return shortest_paths, final_paths

def main(start_end_points, number_of_vehicles, number_of_nodes, original_matrix, matrix_with_reduced_edges, covered_edges):
    """
    Main function which calculates the time needed the algorithm to run and the total safe made by the algorithm

    :param start_end_points: list of tuples where start_end_points[index][0] is the start point for the current vehicle and start_end_points[index][1] is the end point for the vehicle
    :param number_of_vehicles: integer value showing the total input number of vehciles
    :param number_of_nodes: integer value showing the total number of nodes in the input graph structure
    :param original_matrix: adjacency matrix representing the original form of the input graph structure
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    :param covered_edges: set which stores which edges have been covered by at least one vehicle
    :return shortest_paths: list of lists, where shortest_paths[index] shows the best found path for vehicle index
    :return final_paths: list of lists, where final_paths[index] stores the next node in the best path for vehicle index - look at the Elzbieta Futkowska's report on page 29 - pseudo code for the Potential Field Method inspired solution

    :return: total elapsed time from running the algorithm to finishing the calculations
    :return total_safe: float value showing the total safe made by the algorithm considering platooning
    """

    start_end_points = copy.deepcopy(start_end_points)
    vehicles_with_path = []
    shortest_paths = []
    final_paths = []

    costs_without_platooning = dict()

    start = time.time()

    # First run of the algorithm to calculate the best paths for all vehicles without considering platooning
    for i in range(number_of_vehicles):
        dstar_object = DStar(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        real_path = dstar_object.run()

        if real_path == []:
            shortest_paths.append([])
            final_paths.append([])
            continue

        vehicles_with_path.append(i)
        shortest_path_cost = dstar_object.h[start_end_points[i][0]]
        shortest_paths.append(real_path)
        final_paths.append([start_end_points[i][0]])
        costs_without_platooning[i] = shortest_path_cost

    vehicles = vehicles_with_path

    # Second run of the algorithm to calculate the best paths for all vehicles, considering platooning and saving all the edges that are covered by two or more vehicles
    while vehicles:

        for t in vehicles_with_path:
            
            if start_end_points[t][0] == start_end_points[t][1]:
                continue
            
            final_paths[t].append(shortest_paths[t][1])

            # When reducing the weight of a given edge we should be careful for the direction of the edge as D* search works backwards - from the goal to the start node
            if (start_end_points[t][0], shortest_paths[t][1]) not in covered_edges:
                matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]] *= 0.8
                matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]] = round(matrix_with_reduced_edges[shortest_paths[t][1]][start_end_points[t][0]], 2)
                covered_edges.add((start_end_points[t][0], shortest_paths[t][1]))

            start_end_points[t] = (shortest_paths[t][1], start_end_points[t][1])

            if start_end_points[t][0] == start_end_points[t][1]:
                vehicles.remove(t)
            shortest_paths, _ = shortest_paths_for_all_vehicles(number_of_vehicles, number_of_nodes, start_end_points, matrix_with_reduced_edges)
            
    covered_edges = dict()
    matrix_with_reduced_edges = copy.deepcopy(original_matrix)

    # Reduce the weight by 20% of every edge which is covered by two or more vehicles
    for final_path in final_paths:
        if final_path == []:
            continue
        for j in range(len(final_path) - 1):
            if (final_path[j], final_path[j+1]) not in covered_edges:
                covered_edges[(final_path[j], final_path[j+1])] = False
            else:
                if covered_edges[(final_path[j], final_path[j+1])] == False:
                    covered_edges[(final_path[j], final_path[j+1])] = True
                    matrix_with_reduced_edges[final_path[j]][final_path[j+1]] *= 0.8
                    matrix_with_reduced_edges[final_path[j]][final_path[j+1]] = round(matrix_with_reduced_edges[final_path[j]][final_path[j+1]], 2)

    total_safe = 0
    ind = 0

    # Calculate the final cost for going through the best found paths, using a copy of the original matrix where every edge covered by a vehicle in a platoon has a reduced weight by 20%
    for final_path in final_paths:
        if final_path == []:
            ind += 1
            continue
        current_cost = 0
        for j in range(len(final_path) - 1):
            current_cost += matrix_with_reduced_edges[final_path[j]][final_path[j+1]]
        total_safe += (costs_without_platooning[ind] - current_cost)
        ind += 1

    end = time.time()

    return round(end - start, 3), round(total_safe, 2)