import copy
import enlib as el
import osmnx
import matplotlib.pyplot as plt
import osmnx.simplification
import osmnx.distance
import networkx as nx
from tqdm import tqdm
from math import sin, cos, sqrt, exp

D_PRES = 7
PLACE_NAME = "University of Toronto"  # default place
PADDING = 50                          # default padding
RADIUS_BASE = 100
OSM_CRS_EPSG = "EPSG:4326"    # OSM EPSG
                              # https://spatialreference.org/ref/epsg/?search={place_name}&srtext=Search

NODE_COLUMNS = [
    "x",
    "y"
    # "street_count"
]

EDGE_COLUMNS = [
    # "lanes",
    # "oneway",
    "length"
]

BUILDING_COLUMNS = [
    "name",
    "addr:postcode",
    "building",
    "building:levels",
    "height",
    "layer",
    "website",
    "capacity"
]


def get_place_area_km2(place_name, epsg):
    area = osmnx.geocode_to_gdf(place_name)

    return (area["geometry"].to_crs(epsg).area.values[0]) / 10**6
                                    # area in km^2 of place
            # area.at[0, "geometry"].to_crs(epsg).area / 10**6

def show_place_nodes_edges_data(place_name, epsg = "", boundary_buffer_length = 0):
    """
    boundary_buffer_length : distance around place's polygon to add, useful for
                             getting shortest paths that may take routes outside
                             the region of interest

    either both epsg and boundary_buffer_length must be set explicitly, or both default
    """
    nodes, edges = osmnx.graph_to_gdfs(get_place_graph(place_name, epsg, boundary_buffer_length))

    print(nodes[NODE_COLUMNS].head())
    print(edges[EDGE_COLUMNS].head())

def get_place_graph(place_name, epsg = "", boundary_buffer_length = 0):
    """
    either both epsg and boundary_buffer_length must be set explicitly, or both default
    """
    if(boundary_buffer_length > 0):
        return osmnx.graph_from_polygon(
                        get_place_area(place_name, epsg, boundary_buffer_length).at[0, "geometry"],
                        network_type="bike"
                        )
    else:
        return osmnx.graph_from_place(place_name)

def get_place_area(place_name, epsg, boundary_buffer_length):
    place_polygon = osmnx.geocode_to_gdf(place_name).to_crs(epsg)
    place_polygon["geometry"] = place_polygon.buffer(boundary_buffer_length)
    place_polygon = place_polygon.to_crs(OSM_CRS_EPSG)
    return place_polygon

def show_place_basic(place_name):
    area = osmnx.geocode_to_gdf(place_name)

    parks = osmnx.features_from_place(
        place_name,
        {
            "leisure": "park",
            "landuse": "grass",
        },
    )

    water = osmnx.features_from_place(
        place_name,
        {
            "natural": "water",
            "waterway": "*",
        },
    )

    # water["waterway"].value_counts()

    buildings = osmnx.features_from_place(
        place_name,
        {"building": True},
    )

    edges = osmnx.graph_to_gdfs(osmnx.graph_from_place(place_name), nodes = False, edges = True)

    figure, ax = plt.subplots(figsize=(12,8))
    area.plot(ax=ax, facecolor="black")
    parks.plot(ax=ax, facecolor="green")
    water.plot(ax=ax, facecolor="blue")
    edges.plot(ax=ax, linewidth=1, edgecolor="dimgray")
    buildings.plot(ax=ax, facecolor="silver", alpha=0.7)

    plt.show()

def show_network_stats(place_name, epsg, boundary_buffer_length):
    # converting to target CRS for network analysis
    graph = osmnx.project_graph(get_place_graph(place_name, boundary_buffer_length), epsg)

    # figure, ax = osmnx.plot_graph(graph)

    print(osmnx.basic_stats(graph))

    convex_hull = osmnx.graph_to_gdfs(graph, nodes = False, edges = True).unary_union.convex_hull
    print(osmnx.basic_stats(graph, area=convex_hull.area))

def show_place_adv(place_name, epsg, boundary_buffer_length):
    area_to_show = osmnx.geocode_to_gdf(place_name).to_crs(epsg)
    area_to_search = copy.deepcopy(area_to_show)
    area_to_show["geometry"] = area_to_show.buffer(boundary_buffer_length)
    area_to_search["geometry"] = area_to_search.buffer(abs(boundary_buffer_length - PADDING))
    area_to_show = area_to_show.to_crs(OSM_CRS_EPSG)
    area_to_search = area_to_search.to_crs(OSM_CRS_EPSG)
    area_polygon = area_to_search.at[0, "geometry"]

    parks = osmnx.features_from_polygon(
        area_polygon,
        {
            "leisure": "park",
            "landuse": "grass",
        },
    )

    water = osmnx.features_from_polygon(
        area_polygon,
        {
            "natural": "water",
            "waterway": "*",
        },
    )

    buildings = osmnx.features_from_polygon(
        area_polygon,
        {"building": True},
    )

    graph = osmnx.graph_from_polygon(area_polygon, network_type="bike")
    edges = osmnx.graph_to_gdfs(graph, nodes = False, edges = True)

    figure, ax = plt.subplots(figsize=(12,8))
    area_to_show.plot(ax=ax, facecolor="black")
    parks.plot(ax=ax, facecolor="green")
    water.plot(ax=ax, facecolor="blue")
    edges.plot(ax=ax, linewidth=1, edgecolor="dimgray")
    buildings.plot(ax=ax, facecolor="silver", alpha=0.7)

    return (ax, graph)

def get_avaliable_building_data(place_name, epsg, boundary_buffer_length):
    area_to_show = osmnx.geocode_to_gdf(place_name).to_crs(epsg)
    area_to_search = copy.deepcopy(area_to_show)
    area_to_show["geometry"] = area_to_show.buffer(boundary_buffer_length)
    area_to_search["geometry"] = area_to_search.buffer(abs(boundary_buffer_length - PADDING))
    area_to_show = area_to_show.to_crs(OSM_CRS_EPSG)
    area_to_search = area_to_search.to_crs(OSM_CRS_EPSG)

    buildings = osmnx.features_from_polygon(
        area_to_search.at[0, "geometry"],
        {"building": True},
    )

    result = []
    for x in buildings.values:
        if float(x[31]) > 0:  # building height
            result.append((x[2],x[31]))  # bulding name, height
    
    print("WARNING: building data may contain inconsistent units.")
    return result

def find_island_nodes(nodes, edges, start=0):
    """Removes nodes that do not have edges connected."""

    # Adjacency list trades memory for speed
    adj_list = dict()

    for edge in edges:
        if edge[0] in adj_list.keys():
            adj_list[edge[0]].append(edge[1])
        else:
            adj_list[edge[0]] = [edge[1]]
    
    # Breadth First Traversal through graph to find all exporable nodes
    # through the start node

    reachable = []

    search_queue = [start]
    while search_queue:
        search_node = search_queue.pop(0)

        if search_node not in reachable:
            reachable.append(search_node)
            search_queue.extend(adj_list[search_queue])
    
    
    return reachable

def remove_no_fly_zones(tgt_x_y_r, org_graph):
    print("Removing no-fly zones...")
    pbar = tqdm(total=len(tgt_x_y_r))
    for x, y, r in tgt_x_y_r:
        r += RADIUS_BASE
        node, dst = osmnx.distance.nearest_nodes(org_graph, x, y, return_dist=True)
        while dst < r:
            org_graph.remove_node(node)
            node, dst = osmnx.distance.nearest_nodes(org_graph, x, y, return_dist=True)
        edge, dst = osmnx.distance.nearest_edges(org_graph, x, y, return_dist=True)
        while dst < r:
            org_graph.remove_edge(*edge)
            edge, dst = osmnx.distance.nearest_edges(org_graph, x, y, return_dist=True)
        pbar.update()
    pbar.close()
    print("Removed no-fly zones!")

def get_decomposed_network(place_name, epsg, boundary_buffer_length, no_fly_zones, simplification_tolerance=1):
    """
    returns (nodes, edges, UID_to_ind, ind_to_UID) := 
    ([index -> (x,y) ...], 
     [u -> [(v,length) ...]], 
     {UID: index ...}, 
     [index -> UID ...])
    for given place name, epsg, boundary buffer length, and shifts
    coordinate system to the centroid for nodes.

    need to pass a wind calibration function:
      wind_func(sx, sy, dx, dy) -> (V_w_hd, V_w_lt)
    safety check flag ensures uniqueness is preserved in the process.
    simplification_tolerance accepts integral number to simplify graph under.
    """
    print("Getting data from server...")
    place_polygon = osmnx.geocode_to_gdf(place_name).to_crs(epsg)
    place_polygon["geometry"] = place_polygon.buffer(boundary_buffer_length).to_crs(OSM_CRS_EPSG)
    graphB = osmnx.project_graph(osmnx.graph_from_polygon(place_polygon.at[0, "geometry"], network_type="bike"), epsg)
    graphD = osmnx.project_graph(osmnx.graph_from_polygon(place_polygon.at[0, "geometry"], network_type="drive"), epsg)
    if simplification_tolerance > 0:
        graphB = osmnx.simplification.consolidate_intersections(graphB, tolerance=simplification_tolerance)
        graphD = osmnx.simplification.consolidate_intersections(graphD, tolerance=simplification_tolerance)
    print("Got data from server!\nDecomposing graph network...")
    merged_graph = nx.compose(graphD, graphB)
    edge_data = {
        e: min(graphB.edges[e]["length"], graphD.edges[e]["length"]) for e in graphB.edges & graphD.edges
    }
    nx.set_edge_attributes(merged_graph, edge_data, "length")
    graphB = merged_graph
    remove_no_fly_zones(no_fly_zones, graphB)
    # ----------------------------------
    # graphB holds bike + drive network.
    # graphD holds drive network.
    # We will make nodes set from graphB,
    # edges from graphD, dedges from
    # graphB.
    # ----------------------------------
    nodesB, edgesB = osmnx.graph_to_gdfs(graphB)
    edgesD = osmnx.graph_to_gdfs(graphD, nodes=False, edges=True)
    xl, yl, uidl = list(nodesB["x"].values), list(nodesB["y"].values), list(nodesB["osmid_original"].values)
    avg_x = round(sum(x for x in xl) / len(xl), D_PRES)
    avg_y = round(sum(y for y in yl) / len(yl), D_PRES)
    for i in range(len(xl)):
        xl[i] = round((xl[i] - avg_x), max(D_PRES - 3, 0))
    for i in range(len(yl)):
        yl[i] = round((yl[i] - avg_y), max(D_PRES - 3, 0))
    min_x, min_y = min(x for x in xl), min(y for y in yl)
    max_x, max_y = max(x for x in xl), max(y for y in yl)
    ulb, vlb, lenlb = edgesB["u_original"].values, edgesB["v_original"].values, edgesB["length"].values
    uld, vld, lenld = edgesD["u_original"].values, edgesD["v_original"].values, edgesD["length"].values
    print("Graph network decomposed!\nBuilding internal nodes structure...")
    UID_to_ind = {}
    ind_to_UID = []
    nodesl = []
    gc = 0
    for i in range(len(xl)):
        if type(uidl[i]) == list:
            for id in uidl[i]:
                UID_to_ind[id] = gc
        else:
            UID_to_ind[uidl[i]] = gc
        ind_to_UID.append(uidl[i])
        nodesl.append((xl[i], yl[i]))
        gc += 1
    print("Internal nodes structure built!\nBuilding internal edges structure & calibrating winds...")
    edgesl, dedges = [[] for _ in range(gc)], [[] for _ in range(gc)]
    u_ind, v_ind, length = -1, -1, -1
    sx, sy, dx, dy = 0, 0, 0, 0
    x_coeff = 30 / max(abs(max_x), abs(min_x))
    y_coeff = 30 / max(abs(max_y), abs(min_y))
    DEDGE_WORK = []
    fsx, fsy, fdx, fdy = 0, 0, 0, 0
    delta_x, delta_y, delta_norm = 0, 0, 0
    fx, fy, fnorm = 0, 0, 0
    V_w_hd, V_w_lt = 0, 0
    x, y, mul_y, mul_x = 0, 0, 0, 0
    truck_speed, truck_epm, T, Pv, rho = 0, 0, 0, 0, 0
    MAX_TRUCK_SPEED, BASE_TRUCK_SPEED = el.MAX_TRUCK_SPEED, el.BASE_TRUCK_SPEED
    BASE_TEMP, TEMP_FLUC_COEFF, REL_HUMIDITY = el.BASE_TEMP, el.TEMP_FLUC_COEFF, el.REL_HUMIDITY
    QUAD_A, QUAD_B, QUAD_C = el.QUAD_A, el.QUAD_B, el.QUAD_C
    for i in range(len(uld)):
        if not ((uld[i] in UID_to_ind) and (vld[i] in UID_to_ind)):
            continue
        u_ind = UID_to_ind[uld[i]]
        v_ind = UID_to_ind[vld[i]]
        if u_ind == v_ind:    # removing cyclic edges.
            continue
        length = round(lenld[i], max(D_PRES - 3, 0))
        sx, sy = nodesl[u_ind]
        dx, dy = nodesl[v_ind]
        x = x_coeff * (sx + dx)
        y = y_coeff * (sy + dy)
        # ---------------------------
        # Change truck velocity vector field below only.
        # ---------------------------
        mul_y = abs(cos(3+(y/6)))
        mul_x = abs(cos(5+(x/6)))
        truck_speed = round(BASE_TRUCK_SPEED + MAX_TRUCK_SPEED * 0.0003 * (mul_y * x * x + mul_x * y * y), 2)
        # ---------------------------
        truck_epm = QUAD_A * truck_speed + QUAD_B
        truck_epm *= truck_epm
        truck_epm = (truck_epm + QUAD_C) / 1000   # J/m
        edgesl[u_ind].append((v_ind, length, truck_epm * length, truck_speed))
    for j in range(len(ulb)):
        if not ((ulb[j] in UID_to_ind) and (vlb[j] in UID_to_ind)):
            continue
        u_ind = UID_to_ind[ulb[j]]
        v_ind = UID_to_ind[vlb[j]]
        if u_ind == v_ind:    # removing cyclic edges.
            continue
        length = round(lenlb[j], max(D_PRES - 3, 0))
        sx, sy = nodesl[u_ind]
        dx, dy = nodesl[v_ind]
        # ---------------------------
        # Change head wind vector field below only.
        # ---------------------------
        fsx = (sx - 100) / 50
        fsy = sx + sy
        fdx = (dx - 100) / 50
        fdy = dx + dy
        # ---------------------------
        delta_x = dx - sx
        delta_y = dy - sy
        delta_norm = sqrt(delta_x * delta_x + delta_y * delta_y)
        fx = (fsx + fdx) / 2
        fy = (fsy + fdy) / 2
        fnorm = sqrt(fx * fx + fy * fy)
        # max head wind speed: 7 m/s.
        # print(delta_norm, fnorm)
        V_w_hd = 7 * (fx * delta_x + fy * delta_y) / (delta_norm * fnorm)
        # max lateral wind speed: 2 m/s.
        V_w_lt = sin(sx + sy + dx + dy) * 2
        x = x_coeff * (sx + dx)
        y = y_coeff * (sy + dy)
        T = BASE_TEMP + TEMP_FLUC_COEFF * (sin(x) + sin(y))
        Pv = REL_HUMIDITY * 610.78 * exp((17.27 * T) / (T + 237.3))
        rho = ((101325 - Pv) * 0.0034837139 + Pv * 0.0021668274) / (T + 273.15)
        DEDGE_WORK.append((u_ind, len(dedges[u_ind]), rho, V_w_hd, V_w_lt))
        dedges[u_ind].append((v_ind, length))
    el.fill_edge_data(dedges, DEDGE_WORK)
    print("Built internal edges structure & calibrated winds!")
    return (nodesl, edgesl, dedges, UID_to_ind, ind_to_UID)

"""
To analyse OpenStreetMap data over large areas, it is often more efficient and meaningful to download the data all at once,
instead of separate queries to the API. Such data dumps from OpenStreetMap are available in various file formats, OSM Protocolbuffer
Binary Format (PBF) being one of them. Data extracts covering whole countries and continents are available, for instance, at 
download.geofabrik.de

Pyrosm is a Python package for reading OpenStreetMap data from PBF files into geopandas.GeoDataFrames. Pyrosm makes it easy to extract
road networks, buildings, Points of Interest (POI), landuse, natural elements, administrative boundaries and much more - similar to OSMnx,
but taylored to analyses of large areas. While OSMnx reads the data from the Overpass API, pyrosm reads the data from a local PBF file.

Can use pandana library for more efficient network analysis
https://pyrosm.readthedocs.io/en/latest/graphs.html#working-with-graphs
routing resistance
"""

# amazon = osmnx.features_from_bbox(-0.5795, -1.7232, -52.4474, -50.9505, {"waterway" : "river"})
# amazon['simplegeom'] = amazon.simplify(tolerance=20000)
# amazon = amazon.set_geometry('simplegeom')
