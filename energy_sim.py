import geolib as gl
import enlib as el
import matplotlib.pyplot as plt
import shapely.geometry as shp
import random
import sim_helper

PLACE_NAME = "University of Toronto"
TARGET_CRS_EPSG = "EPSG:3348" # Canadian EPSG
BOUNDARY_BUFFER_LENGTH = 500  # default boundary buffer
TYPE = "walk" # "drive"

nodes, edges = gl.get_decomposed_network(PLACE_NAME, TARGET_CRS_EPSG, BOUNDARY_BUFFER_LENGTH, TYPE, 
                                         safety_check=True, simplification_tolerance=5)

eh = el.EnergyHelper(nodes, edges, 10**(-2), gen_plot_data=True)


node_count = max(nodes.keys())
vertices = [0] * node_count
for i in range(node_count):
    vertices[i] = nodes[i]

delivery_point_count = 10
delivery_indices = []

for i in range(delivery_point_count):
    delivery_indices.append(random.randrange(0, node_count))

# print(delivery_indices)
# print(edges)

print(sim_helper.convert_to_adj_list(eh))

eh.plot_network()
plt.show()

