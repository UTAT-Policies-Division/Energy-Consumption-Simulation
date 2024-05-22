import geolib as gl
import enlib as el
import matplotlib.pyplot as plt
import random
import geopandas
import shapely.geometry as shp
import shapely
import networkx as nx

PLACE_NAME = "University of Toronto"
TARGET_CRS_EPSG = "EPSG:3348" # Canadian EPSG
BOUNDARY_BUFFER_LENGTH = 500  # default boundary buffer
TYPE = "bike" # "drive"

# gl.show_place_adv(PLACE_NAME, TARGET_CRS_EPSG, BOUNDARY_BUFFER_LENGTH)
nodes, edges, UID_to_ind, ind_to_UID = gl.get_decomposed_network(PLACE_NAME, 
                                                                 TARGET_CRS_EPSG, 
                                                                 BOUNDARY_BUFFER_LENGTH, 
                                                                 TYPE, 
                                                                 safety_check=True, 
                                                                 simplification_tolerance=5)
# nodes = [(0,0), (1,0), (1,1), (5,0), (2,3)]
# edges = [[(1, 10.0)], 
#          [(0, 10.0), (2, 10.0), (3, 40.0)], 
#          [(1, 10.0), (4, 30.5)], 
#          [(1, 40.0)], 
#          [(2, 30.5)]]
eh = el.EnergyHelper(nodes, edges, 10**(-2), gen_plot_data=True)
# print(eh.classify_turn_angle(0, 1, 3))
# print(eh.edge_exists(0, 3))
eh.plot_network()
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
