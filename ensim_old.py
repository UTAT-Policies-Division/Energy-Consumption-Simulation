import math
import sys
import copy
import numpy as np
import geolib as gl
import enlib as el
import matplotlib.pyplot as plt
import shapely.geometry as shp
import random
import sim_helper
import min_span_tree
import pickler as pkl

def energy_delivery_points(delivery_pt0:int, delivery_pt1:int):
    """djikstra's algorithm implemented between 2 delivery points"""
    global delivery_indices, map_adj_list


    return dijkstras(
        map_adj_list, 
        delivery_nodes[delivery_pt0], 
        delivery_nodes[delivery_pt1], 
        energy_map_points
    )

def dijkstras(adjacency_list: list, start: int, dest: int, energy) -> list:
    cardinality = len(adjacency_list)
    distances = np.ones(cardinality) * np.inf
    distances[start] = 0

    connections = np.ones(shape=(cardinality), dtype=int) * -1
    check_stack = [start]
    visited = np.zeros(shape=(cardinality), dtype=bool)

    while check_stack:
        n0 = check_stack.pop()

        if not visited[n0]:
            visited[n0] = True

            for n1 in adjacency_list[n0]:
                test_distance = distances[n0] + energy(n0, n1)

                if test_distance < distances[n1]:
                    distances[n1] = test_distance
                    connections[n1] = n0
                
                check_stack.append(n1)
    
    path = [dest]
    while path[0] != start or len(path) < len(adjacency_list):
        if(connections[path[0]] == -1):
            break
        path.insert(0, connections[path[0]])
    return distances[dest], path

def euclidean_distance(vector_a, vector_b):
    """Returns the Euclidean Distance between vectors v1 and v2"""
    d0 = vector_a[0] - vector_b[0]
    d1 = vector_a[1] - vector_b[1]
    return math.sqrt(d0 * d0 + (d1 * d1))

def energy_map_points(map_vertex0:int, map_vertex1:int) -> float:
    """Energy Required to travel on an edge on map"""
    global map_nodes
    n0 = map_nodes[map_vertex0]
    n1 = map_nodes[map_vertex1]

    return euclidean_distance(n0, n1)

# For Repeatable Runs
seed = random.randrange(sys.maxsize)
random.seed(seed)
print("Random Seed: ", seed)

UOFT = "University of Toronto"
MANHATTAN = "Manhattan"
PLACE_NAME = UOFT
TORONTO_CRS_EPSG = "EPSG:3348"
LONG_ISLAND_CRS_EPSG = "EPSG:32118"
TARGET_CRS_EPSG = TORONTO_CRS_EPSG
BOUNDARY_BUFFER_LENGTH = 500  # default boundary buffer
NUM_STOPS = 200
GET_MONTH_INDEX = {"January":0,
                   "February":1,
                   "March":2,
                   "April":3,
                   "May":4,
                   "June":5,
                   "July":6,
                   "August":7,
                   "September":8,
                   "October":9,
                   "November":10,
                   "December":11}

def RH(isMorning, month):
    if isMorning:
        if month == 0:
            return 0.66
        elif month == 1:
            return 0.65
        elif month == 2:
            return 0.64
        elif month == 3:
            return 0.64
        elif month == 4:
            return 0.73
        elif month == 5:
            return 0.76
        elif month == 6:
            return 0.75
        elif month == 7:
            return 0.77
        elif month == 8:
            return 0.78
        elif month == 9:
            return 0.74
        elif month == 10:
            return 0.71
        else:
            return 0.69
    else:
        if month == 0:
            return 0.55
        elif month == 1:
            return 0.53
        elif month == 2:
            return 0.50
        elif month == 3:
            return 0.45
        elif month == 4:
            return 0.52
        elif month == 5:
            return 0.55
        elif month == 6:
            return 0.53
        elif month == 7:
            return 0.54
        elif month == 8:
            return 0.56
        elif month == 9:
            return 0.55
        elif month == 10:
            return 0.57
        else:
            return 0.59


isMorning = False
Month = "March"
el.init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
                base_temperature=20, temp_flucts_coeff=3, drone_speed=18,
                relative_humidity=RH(isMorning,GET_MONTH_INDEX[Month]))
eh = pkl.load_decomposed_network(
    PLACE_NAME, 
    TARGET_CRS_EPSG, 
    BOUNDARY_BUFFER_LENGTH, 
    simplification_tolerance=1
)
print(eh.nodes)

nodes = eh.nodes
adj_list = sim_helper.convert_to_adj_list(eh)
map_node_count = len(nodes)

#------SIM CODE-------#

# Generate an abstraction layer for delivery points
delivery_point_count = 10
delivery_indices = []

for i in range(delivery_point_count):
    delivery_indices.append(random.randrange(0, map_node_count))
print(delivery_indices)
# (Global) Problem-Related Variables
map_nodes = nodes
map_adj_list = adj_list

# A list where each value corresponds to the map coordinate of each 
# delivery point.
delivery_nodes = delivery_indices
delivery_adj_list = []

# Adjacency List for delivery points, we assume every delivery point 
# can be accessed by every other delivery point.
for i in range(delivery_point_count):
    adj = []
    for j in range(delivery_point_count):
        adj.append(j)
    delivery_adj_list.append(adj)

# Starting Vertex
warehouse = 0

# Generate Distance Matrix for delivery layer
distance_matrix = np.zeros(shape=(delivery_point_count, delivery_point_count))

for i in range(delivery_point_count):
    for j in range(delivery_point_count):
        distance_matrix[i, j], _ = energy_delivery_points(i, j)

# Solve GTSP using distance matrix
from python_tsp.heuristics import solve_tsp_lin_kernighan as tsp_solve
tour, i = tsp_solve(distance_matrix, warehouse)

print(tour, i)
tour.append(warehouse) # Fix formatting
map_tour = []
for node in tour:
    map_tour.append(delivery_nodes[node])
print(map_tour)
    

# Plotting
sim_helper.plot_graph_adj(nodes, adj_list)
sim_helper.plot_tour(nodes, map_tour, delivery_point_count)

for i in range(delivery_point_count):
    cost, path = dijkstras(map_adj_list, map_tour[i], map_tour[i+1], energy_map_points)
    # print((map_tour[i], map_tour[i+1]), (cost, path))
    sim_helper.plot_tour(nodes, path, len(path)-1, edge_color='red', node_color='lightcoral')

plt.show()