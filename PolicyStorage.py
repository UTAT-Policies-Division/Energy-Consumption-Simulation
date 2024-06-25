import random
import osmnx


# Constants
NUM_BUILDINGS = 10
MAX_PAYLOAD = 500  # kg
# Maximum packages a building can request
MAX_DEMAND_PER_BUILDING = 5  
# Total number of packages in the simulation
NUM_PACKAGES = 100  
# metres
MAX_BUILDING_REACH = 1000  
# m/s
OPTIMAL_SPEED = 10  
# in metres
OPTIMAL_ALTITUDE = 100  

TAGS = [{},
        {'aeroway': ['aerodrome', 'aircraft_crossing', 'apron', 'gate',
                     'hangar', 'helipad', 'navigationaid', 'runway',
                     'spaceport', 'taxiway', 'terminal', 'windsock'],
         'building': ['stadium', 'government'],
         'boundary': 'protected_area',
         'landuse': 'military',
         'military': ['academy', 'airfield', 'base', 'bunker', 'barracks',
                      'checkpoint', 'danger_area', 'nuclear_explosion_site',
                      'obstacle_course', 'office', 'range', 'school',
                      'training_area', 'trench'],
         'office': 'government',
         'power': ['generator', 'planet', 'substation']},
        {'highway': ['motorway', 'trunk'],
         'building': ['firestation', 'hospital', 'prison']},
        {'amenity': ['school', 'college', 'university'],
         'public_transport': 'station'}]


class dictGraph:
    def __init__(self):
        # Vertice
        self.V = {}  
         # Edge
        self.E = {} 

    def add_vertex(self, vertex):
        if vertex.label not in self.V:
            self.V[vertex.label] = vertex
            self.E[vertex.label] = []

    def add_edge(self, from_label, to_label):
        if from_label in self.V and to_label in self.V:
            self.E[from_label].append(to_label)  # Edge direct

class Vertex:
    def __init__(self, label):
        self.label = label
        self.delivery_arrived = False

class PolicyData:
    def __init__(self):
        self.NUM_BUILDINGS = NUM_BUILDINGS
        self.MAX_PAYLOAD = MAX_PAYLOAD
        self.MAX_DEMAND_PER_BUILDING = MAX_DEMAND_PER_BUILDING
        self.NUM_PACKAGES = NUM_PACKAGES
        self.MAX_BUILDING_REACH = MAX_BUILDING_REACH
        self.OPTIMAL_SPEED = OPTIMAL_SPEED
        self.OPTIMAL_ALTITUDE = OPTIMAL_ALTITUDE
        self.REGION_POLICY = {
            'MAX_SPEED': self.OPTIMAL_SPEED,
            'MAX_ALTITUDE': self.OPTIMAL_ALTITUDE,
            'NO_FLY_ZONES': {'set 1': [],
                             'set 2': [],
                             'set 3': [],
                             'set 4': []}  # Dictionary of no-fly zone osmids
            # 'NO_FLY_ZONES': {'set 1': [278112877],
            #                  'set 2': [278112877, 3695755, 109337830, 11077079,
            #                            7554971, 357620387, 271885447, 702540318,
            #                            46177116, 34633854, 8398106, 702472511,
            #                            129835611, 6615147359],
            #                  'set 3': [],
            #                  'set 4': []}  # Dictionary of no-fly zone osmids
        }

        self.truck_routes = dictGraph()  # Truck routes
        self.drone_routes = dictGraph()  # Drone routes

        self.Sv_truck_routes = set()  # Set of vertices closest to v in graph truck_routes
        self.vertices_building = []  # Collection of building vertices

# Example probability distribution function
def sample_building_selection_distribution():

    return random.randint(1, NUM_BUILDINGS)

#  demand per building random variable func
def sample_demand_per_building():
    return random.randint(1, MAX_DEMAND_PER_BUILDING)

# # Example use
# policy_data = PolicyData()

# # vertice define
# source_vertex = Vertex('source')
# building_vertex = Vertex('destination')

# # Vertices+Graph
# policy_data.truck_routes.add_vertex(source_vertex)
# policy_data.drone_routes.add_vertex(building_vertex)

# # If required add edges
# policy_data.truck_routes.add_edge(source_vertex.label, building_vertex.label)

# # Update
# policy_data.Sv_truck_routes.add(source_vertex.label)
# policy_data.vertices_building.append(building_vertex)

# Define no-fly zones in REGION_POLICY if needed
def no_fly_zones(zone, policy_object, epsg):
    keys = ['set 1', 'set 2', 'set 3', 'set 4']
    for j in range(len(TAGS)):
        if TAGS[j]:
            geoms =  osmnx.project_gdf(osmnx.features_from_place(zone, TAGS[j]), epsg)['geometry']
            for i in range(len(geoms)):
                center = geoms[i].centroid
                cx, cy = center.x, center.y
                lx, ly, ux, uy = geoms[i].bounds
                rad = (max(cx - lx, ux - cx) + max(cy - ly, uy - cy)) / 2
                policy_object.REGION_POLICY['NO_FLY_ZONES'][keys[j]].append((cx, cy, rad))
            # tuples = list(features.axes[0])
            # osmids = [tuple[1] for tuple in tuples]
            # policy_object.REGION_POLICY['NO_FLY_ZONES'][keys[j]].extend(osmids)

# no_fly_zones(["Manhattan, United States"], policy_data, TAGS)

# # Policy Data 
# print(f"# of Buildings: {policy_data.NUM_BUILDINGS}")
# print(f"Maximum payload: {policy_data.MAX_PAYLOAD} kg")
# print(f"Each Building Maximum demand: {policy_data.MAX_DEMAND_PER_BUILDING}")
# print(f"# of packages: {policy_data.NUM_PACKAGES}")
# print(f"Building reach maximum: {policy_data.MAX_BUILDING_REACH} meters")
# print(f"Speed-Optimum: {policy_data.OPTIMAL_SPEED} m/s")
# print(f"Altitude-Optimum: {policy_data.OPTIMAL_ALTITUDE} meters")
# print(f"Policy Region: {policy_data.REGION_POLICY}")
# print(f"Graph vertices Truck: {list(policy_data.truck_routes.V.keys())}")
# print(f"Graph vertices Drone: {list(policy_data.drone_routes.V.keys())}")
