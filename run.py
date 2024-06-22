import geolib as gl
import enlib as el
import matplotlib.pyplot as plt
import random
import geopandas
import shapely.geometry as shp
import shapely
import networkx as nx

UOFT = "University of Toronto"
MANHATTAN = "Manhattan"
TORONTO_CRS_EPSG = "EPSG:3348"
LONG_ISLAND_CRS_EPSG = "EPSG:32118"
BOUNDARY_BUFFER_LENGTH = 500  # default boundary buffer
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

if __name__ == '__main__':
  PLACE_NAME = MANHATTAN
  TARGET_CRS_EPSG = LONG_ISLAND_CRS_EPSG
  isMorning = False
  Month = "March"
  el.init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
                 base_temperature=25, temp_flucts_coeff=3, drone_speed=14,
                 relative_humidity=RH(isMorning,GET_MONTH_INDEX[Month]))
#   nodes, edges, dedges, UID_to_ind, ind_to_UID = gl.get_decomposed_network(PLACE_NAME, 
#                                                                    TARGET_CRS_EPSG, 
#                                                                    BOUNDARY_BUFFER_LENGTH,
#                                                                    simplification_tolerance=1)
#   eh = el.EnergyHelper(nodes, edges, dedges, UID_to_ind, ind_to_UID,
#                        10**(-2), gen_plot_data=True, demand=[])
#   gl.show_place_adv(PLACE_NAME, TARGET_CRS_EPSG, BOUNDARY_BUFFER_LENGTH)
#   nodes = [(0,0), (1,0), (1,1), (5,0), (2,3)]
#   edges = [[(1, 10.0)], 
#            [(0, 10.0), (2, 10.0), (3, 40.0)], 
#            [(1, 10.0), (4, 30.5)], 
#            [(1, 40.0)], 
#            [(2, 30.5)]]
#   print(nodes[0:100])
#   print(edges[0:100])
#   print(eh.classify_turn_angle(0, 1, 3))
#   print(eh.edge_exists(0, 3))
#   eh.save("uoft.pkl")
  NUM_STOPS = 200
  NUM_ALLOCS = 10
  RANGE = float(10000000)   # dummy for now
  eh = el.EnergyHelper.load("uoft.pkl")
#   eh.enforce_graph_connections()
#   eh.demand = [(602, 11.5), (301, 8.25), (0, 0.25), (193, 0.5), (435, 9.25), 
#                (42, 1.75), (115, 1), (56, 0.75), (223, 1.5), (348, 1)]
#   b_d = 1000
#   b_ind = -1
#   for i in range(len(eh.demand)):
#     dem = eh.demand[i][0]
#     if b_d > abs(eh.nodes[dem][0] + 25) + abs(eh.nodes[dem][1] + 50):
#       b_d = abs(eh.nodes[dem][0] + 25) + abs(eh.nodes[dem][1] + 50)
#       b_ind = i
#   print(b_ind, eh.nodes[eh.demand[b_ind][0]])
#   eh.demand.pop(b_ind)
#   eh.append_random_demand(50, cluster_num=0, cluster_jump=0)
  eh.append_random_demand(10, cluster_num=0, cluster_jump=0,
                          drone_only_possible_component=0.6, num_allocs=NUM_ALLOCS)
  src = eh.get_top_right_node()
  eh.init_phermone_system(src, NUM_ALLOCS, R=RANGE)
#   print(eh.edges[0][0])
#   print(max(max(v for v in arr) for arr in eh.n_pherm))
#   for i in range(25, 201, 25):
#     energy, cycle, swp = eh.aco(K=i)
#     print(i, "Energy of plotted cycle in MJ:", round(energy / 10**6, 2))
#     print(cycle)
  energy, cycle, swp = eh.aco(ants_per_iter=1)
#   energy, cycle = eh.aco_truck_only()
  print("Energy of plotted cycle in MJ:", round(energy / 10**6, 2))
  print(cycle)
  eh.plot_cycle(cycle, [])
#   print(cycle, swp)
#   eh.show_swp_string(swp)
#   eh.plot_cycle(cycle, swp)
#   energy, cycle, swp = eh.aco_truck_only()
#   print("Energy of plotted cycle in MJ:", round(energy / 10**6, 2))
#   pth = [eh.demand[0][0]]
#   print(eh.nodes[eh.demand[0][0]])
#   pth.extend(lep_t[0][0])
#   print(eh.total_weight)
#   eh.plot_network(show_drone_only_nodes=False,
#                   show_drone_only_edges=False,
#                   show_demand_nodes=True,
#                   show_demand_local_paths=True,
#                   show_for_all_edges=False,
#                   spec_ind=[],
#                   spec_path=[])
#   el.DRONE_GROUND_SPEED = el.kph_to_mps(30)
#   print(el.power(el.rho_air_std,
#                  el.kgs_to_W(2.5),
#                  el.kph_to_mps(10),
#                  el.kph_to_mps(5)))
#   def func(V, HPS):
#       return el.power(el.rho_air_std,
#                   el.kph_to_mps(V),
#                   el.kgs_to_W(2.5),
#                   el.kph_to_mps(HPS),
#                   el.kph_to_mps(5), CHORD, BETA, SINPSI, COSPSI)
#   el.draw_functions(30,50,5,func,-5,5,3)
  """
  climb/descent speed range: -30 kmh to 25 kmh including wind.
  forward ground speed range: 0 kmh to 170 kmh including wind. 
  """
#   def func(V):
#       return el.power(el.rho_air_std,
#                   el.kph_to_mps(V),
#                   el.kgs_to_W(1.0),
#                   el.kph_to_mps(5.44),
#                   el.kph_to_mps(5), CHORD, BETA, SINPSI, COSPSI)
#   el.draw_function(0,50,10,func)
#   def func(rpm):
#     return el.TH_BET(el.rho_air_std, 2.43, 23.0, 4.25, el.RPM_to_omega(rpm), CHORD, BETA, SINPSI, COSPSI)[1]
#   el.draw_function(0,12000,1000,func)
#   plt.legend(loc='best')
  plt.savefig("pic.png", dpi=700)
  plt.show()


"""
# ax, graph = show_place_adv(PLACE_NAME, TARGET_CRS_EPSG, BOUNDARY_BUFFER_LENGTH)
# nodes, edges = osmnx.graph_to_gdfs(graph)

# cycle_roads = edges.loc[edges["highway"] == "cycleway", :]

places = ["Sidney Smith Hall",
          "Riverdale farm",
          "Westin Harbour Conference Centre",
          "Billy Bishop Toronto City Airport",
          "Tommy Thompson Park"]
print("reading place data...")
places_data = osmnx.geocode_to_gdf(places).to_crs(TARGET_CRS_EPSG)["geometry"]
centroids = []
print("finding centroids...")
for obj in places_data:
    centroids.append(obj.centroid)

# direct polygon graph:
# print("forming region of interest...")
# area = shapely.MultiPoint(centroids).convex_hull.buffer(BOUNDARY_BUFFER_LENGTH)

area = shapely.union_all(centroids).centroid
max_len = 0
print("finding buffer distance...")
for i in range(len(centroids)):
    max_len = max(shapely.distance(centroids[i], area), max_len)
print("buffer distance:", round((max_len + BOUNDARY_BUFFER_LENGTH)/1000, 2), "km")
area = area.buffer(max_len + BOUNDARY_BUFFER_LENGTH)
gdf = geopandas.GeoDataFrame({'geometry': [area]}, crs=TARGET_CRS_EPSG).to_crs(OSM_CRS_EPSG)
area = gdf.at[0, 'geometry']
print("buffering grass features in region of interest...")
parks = osmnx.features_from_polygon(
    area,
    {
        "leisure": "park",
        "landuse": "grass",
    },
)
intersection_parks = []
for obj in parks['geometry']:
    intersection_parks.append(shapely.intersection(area, obj))
parks['geometry'] = intersection_parks
print("buffering water features in region of interest...")
water = osmnx.features_from_polygon(
    area,
    {
        "natural": "water",
        "waterway": "*",
    },
)
intersection_water = []
for obj in water['geometry']:
    intersection_water.append(shapely.intersection(area, obj))
water['geometry'] = intersection_water
print("buffering buildings in region of interest...")
buildings = osmnx.features_from_polygon(
    area,
    {"building": True},
)
intersection_buildings = []
for obj in buildings['geometry']:
    intersection_buildings.append(shapely.intersection(area, obj))
buildings['geometry'] = intersection_buildings
print("plotting region of interest...")
graph = osmnx.graph_from_polygon(area, network_type="all")   # "drive" for truck
nodes, edges = osmnx.graph_to_gdfs(graph, nodes = True, edges = True)
figure, ax = plt.subplots(figsize=(12,8))
gdf.plot(ax=ax, facecolor="black")
parks.plot(ax=ax, facecolor="green")
water.plot(ax=ax, facecolor="blue")
edges.plot(ax=ax, linewidth=1, edgecolor="dimgray")
buildings.plot(ax=ax, facecolor="silver", alpha=0.7)

targets_gdf = geopandas.GeoDataFrame({'geometry': places_data}, crs=TARGET_CRS_EPSG)
targets_gdf.to_crs(OSM_CRS_EPSG).plot(ax=ax, facecolor='red')
target_graph = osmnx.project_graph(graph, TARGET_CRS_EPSG)
target_nodes, target_edges = osmnx.graph_to_gdfs(target_graph, nodes = True, edges = True)

for i in range(len(places)):
    for j in range(i + 1, len(places)):
        origin_node_id = osmnx.nearest_nodes(target_graph, centroids[i].x, centroids[i].y)
        destination_node_id = osmnx.nearest_nodes(target_graph, centroids[j].x, centroids[j].y)
        route = osmnx.shortest_path(target_graph, origin_node_id, destination_node_id, weight="length")
        route_nodes = target_nodes.loc[route]
        route_line = shp.LineString(list(route_nodes.geometry.values))
        route_geom = geopandas.GeoDataFrame(
            {
                "geometry": [route_line],
                "osm_nodes": [route],
            },
            crs=target_edges.crs
        ).to_crs(OSM_CRS_EPSG)
        route_geom.plot(ax=ax, linewidth=2, linestyle='solid', color='red', alpha=0.45)
plt.show()

"""
"""

origin_place = osmnx.geocode_to_gdf("Sidney Smith Hall")
destination_place = osmnx.geocode_to_gdf("Health Sciences Building, University of Toronto")

data = {'names': ['origin', 'destination'], 
        'geometry': [origin_place.at[0, "geometry"], destination_place.at[0, "geometry"]]}
gdf = geopandas.GeoDataFrame(data, crs="EPSG:4326")

gdf.plot(ax=ax, facecolor='red')

n_graph = osmnx.project_graph(graph, "EPSG:3348")
n_edges = osmnx.graph_to_gdfs(n_graph, nodes = False, edges = True)

# shapely.geometry.Point objects
n_origin = (
    origin_place  # fetched geolocation
    .to_crs(n_edges.crs)  # transform to current CRS
    .at[0, "geometry"]  # pick geometry of first row
    .centroid
)

n_destination = (
    destination_place
    .to_crs(n_edges.crs)
    .at[0, "geometry"]
    .centroid
)

origin_node_id = osmnx.nearest_nodes(n_graph, n_origin.x, n_origin.y)
destination_node_id = osmnx.nearest_nodes(n_graph, n_destination.x, n_destination.y)

mod_edges = n_graph.edges
for e in mod_edges:
    mod_edges[e]["weight"] = mod_edges[e]["length"]  # modify this to change weights !

# mod_n_graph  = nx.MultiDiGraph()
# mod_n_graph.add_nodes_from(graph)
# mod_n_graph.add_edges_from(graph.edges)

route = osmnx.shortest_path(n_graph, origin_node_id, destination_node_id, weight="weight")
route_nodes = nodes.loc[route]
route_line = shp.LineString(
    list(route_nodes.geometry.values)
)
route_geom = geopandas.GeoDataFrame(
    {
        "geometry": [route_line],
        "osm_nodes": [route],
    },
    crs=edges.crs
)

route_geom.plot(ax=ax, linewidth=2, linestyle='solid', color='red', alpha=0.45)

# osmnx.plot_graph_route(graph, route, ax=ax, route_linewidth=1, route_color="r")
                       # note using orginal and not translated n_graph to plot

plt.show()
"""
