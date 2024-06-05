import numpy as np
import geolib as gl
import enlib as el
import matplotlib.pyplot as plt
import shapely.geometry as shp
import random
import sim_helper

def energy_delivery_points(delivery_pt0:int, delivery_pt1:int) -> float:
    """djikstra's algorithm implemented between 2 delivery points"""
    global delivery_indices


    return 100

def energy_map_points(map_vertex0:int, map_vertex1:int) -> float:
    """Energy Required to travel on an edge on map"""
    global real_nodes, real_adj_list
    n0 = real_nodes[map_vertex0]
    n1 = real_nodes[map_vertex1]

    return min_span_tree.euclidean_distance(n0, n1)

PLACE_NAME = "University of Toronto"
TARGET_CRS_EPSG = "EPSG:3348" # Canadian EPSG
BOUNDARY_BUFFER_LENGTH = 500  # default boundary buffer
TYPE = "drive" # "drive"

nodes, edges, UID_to_ind, ind_to_UID = gl.get_decomposed_network(PLACE_NAME, TARGET_CRS_EPSG, BOUNDARY_BUFFER_LENGTH, TYPE, 
                                         safety_check=True, simplification_tolerance=5)

eh = el.EnergyHelper(nodes, edges, 10**(-2), gen_plot_data=True)

adj_list = sim_helper.convert_to_adj_list(nodes, edges)
node_count = len(nodes)


delivery_point_count = 10
delivery_indices = []

for i in range(delivery_point_count):
    delivery_indices.append(random.randrange(0, node_count))

print(delivery_indices)


# (Global) Problem-Related Variables
real_nodes = nodes
real_adj_list = adj_list

delivery_nodes = delivery_indices
delivery_adj_list = []
for i in range(delivery_point_count):
    adj = []
    for j in range(delivery_point_count):
        adj.append(j)
    delivery_adj_list.append(adj)

# Starting Vertex
warehouse = 0

import min_span_tree
tour = min_span_tree.tsp_krusal_solve(delivery_nodes, delivery_adj_list, warehouse, energy_map_points)
print(tour)

# PLOT TOUR ON MAP
# Plot Graph from Adjacency Matrix
sim_helper.plot_graph_adj(nodes, adj_list)
for i in range(delivery_point_count):
    n0 = nodes[tour[i]]
    n1 = nodes[tour[i+1]]

    x = [n0[0], n1[0]]
    y = [n0[1], n1[1]]

    plt.plot(x, y, 'g')
    
plt.show()

