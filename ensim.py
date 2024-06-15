import geolib as gl
import enlib as el
import matplotlib.pyplot as plt
import shapely.geometry as shp
import random
import pickler as pkl
import sim_helper

UOFT = "University of Toronto"
MANHATTAN = "Manhattan"
PLACE_NAME = MANHATTAN
TORONTO_CRS_EPSG = "EPSG:3348"
LONG_ISLAND_CRS_EPSG = "EPSG:32118"
TARGET_CRS_EPSG = LONG_ISLAND_CRS_EPSG
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

nodes = eh.nodes
edges = eh.edges

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

