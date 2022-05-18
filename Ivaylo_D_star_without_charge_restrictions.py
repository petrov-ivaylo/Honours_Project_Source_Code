"""
D* search implementing Ivaylo Petrov's idea for heuristically solving the standard Vehicle Platooning problem - without recharging points
@author: Ivaylo Petrov
Comment: The DStar class is a modified version of the huiming zhou's version, which can be found at 
         https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_2D/D_star.py
"""

import copy # Used for copying objects
import heapq    # Used for constructing a priority queue structure
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


def main(start_end_points, number_of_vehicles, number_of_nodes, matrix_with_reduced_edges, covered_edges):
    """
    Main function which calculates the time needed the algorithm to run and the total safe made by the algorithm

    :param start_end_points: list of tuples where start_end_points[index][0] is the start point for the current vehicle and start_end_points[index][1] is the end point for the vehicle
    :param number_of_vehicles: integer value showing the total input number of vehciles
    :param number_of_nodes: integer value showing the total number of nodes in the input graph structure
    :param matrix_with_reduced_edges: adjacency matrix (list of lists of integers) representing the current state of the input graph
    :param covered_edges: set which stores which edges have been covered by at least one vehicle

    :return: total elapsed time from running the algorithm to finishing the calculations
    :return total_safe: float value showing the total safe made by the algorithm considering platooning
    """

    start_end_points = copy.deepcopy(start_end_points)
    best_paths_without_platooning = []

    max_priority_queue = []
    costs_without_platooning = dict()

    start = time.time()

    # First run of the algorithm to calculate the best paths for all vehicles without considering platooning
    for i in range(number_of_vehicles):
        dstar_object = DStar(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        real_path = dstar_object.run()
        
        if real_path == []:
            continue

        shortest_path_cost = dstar_object.h[start_end_points[i][0]]
        heapq.heappush(max_priority_queue, (len(real_path)*(-1), i))
        costs_without_platooning[i] = round(shortest_path_cost, 2)

    for i in range(len(max_priority_queue)):
        best_paths_without_platooning.append(heapq.heappop(max_priority_queue)[1])

    final_paths = [[] for i in range(number_of_vehicles)]
    local_matrix_with_reduced_edges = copy.deepcopy(matrix_with_reduced_edges)

    # Second run of the algorithm to calculate the best paths for all vehicles, considering platooning and saving all the edges that are covered by two or more vehicles
    for i in best_paths_without_platooning:
        dstar_object = DStar(number_of_nodes, matrix_with_reduced_edges, start_end_points[i][0], start_end_points[i][1])
        real_path = dstar_object.run()

        for j in range(len(real_path)-1):
            # We check if the current edge has already been traversed in the same direction only because the vehicle will move this exact direction
            if (real_path[j], real_path[j+1]) not in covered_edges:
                covered_edges[(real_path[j], real_path[j+1])] = False
                # If an edge is covered by a vehicle, then by reducing the edge's weight by 20%, we mark that edge as a potential possibility of platooning for the following vehicles
                matrix_with_reduced_edges[real_path[j+1]][real_path[j]] *= 0.8
                matrix_with_reduced_edges[real_path[j+1]][real_path[j]] = round(matrix_with_reduced_edges[real_path[j+1]][real_path[j]], 2)
            
            # local_matrix_with_reduced_edges represents the original matrix where every edge's weight is reduced by 20% of its initial value if the edge is covered by two or more vehicles
            else:
                if covered_edges[(real_path[j], real_path[j+1])] == False:
                    covered_edges[(real_path[j], real_path[j+1])] = True
                    local_matrix_with_reduced_edges[real_path[j+1]][real_path[j]] *= 0.8
                    local_matrix_with_reduced_edges[real_path[j+1]][real_path[j]] = round(matrix_with_reduced_edges[real_path[j+1]][real_path[j]], 2)


        final_paths[i] = real_path
        
    matrix_with_reduced_edges = copy.deepcopy(local_matrix_with_reduced_edges)
    total_safe = 0

    # Calculate the final cost for going through the best found paths, using a copy of the original matrix where every edge covered by a vehicle has a reduced weight by 20%
    for final_path in range(len(final_paths)):
        if final_paths[final_path] == []:
            continue
        current_vehicle_safe = 0
        for i in range(len(final_paths[final_path]) - 1):
            current_vehicle_safe += round(matrix_with_reduced_edges[final_paths[final_path][i+1]][final_paths[final_path][i]], 2)
            current_vehicle_safe = round(current_vehicle_safe, 2)
        total_safe += (costs_without_platooning[final_path] - current_vehicle_safe)
        total_safe = round(total_safe, 2)

    end = time.time()

    return round(end - start, 3), round(total_safe, 2)