import copy
from numpy import sort
import osmnx
import matplotlib.pyplot as plt
import osmnx.simplification

D_PRES = 7
PLACE_NAME = "University of Toronto"  # default place
PADDING = 50                          # default padding
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

def get_decomposed_network(place_name, epsg, boundary_buffer_length, type, 
                           simplification_tolerance=0, safety_check=False):
    """
    returns (nodes, edges, UID_to_ind, ind_to_UID) := 
    ([index -> (x,y) ...], 
     [u -> [(v,length) ...]], 
     {UID: index ...}, 
     [index -> UID ...])
    for given place name, epsg, boundary buffer length, and shifts
    coordinate system to the centroid for nodes.

    safety check flag ensures uniqueness is preserved in the process.
    """
    print("Getting data from server...")
    graph = osmnx.graph_from_polygon(
                    get_place_area(place_name, epsg, boundary_buffer_length).at[0, "geometry"], 
                    network_type=type)
    if simplification_tolerance > 0:
        graph = osmnx.project_graph(
                    osmnx.simplification.consolidate_intersections(
                        osmnx.project_graph(graph, epsg), tolerance=simplification_tolerance), 
                    OSM_CRS_EPSG)
    nodes, edges = osmnx.graph_to_gdfs(graph)
    print("Got data from server!\nDecomposing graph network...")
    uidl = nodes.axes[0].values
    xl = nodes["x"].values
    yl = nodes["y"].values
    if safety_check:
        lst = sort(xl)
        lc = round(lst[1], D_PRES) - round(lst[0], D_PRES)
        for j in range(1,lst.size):
            lc = min(lc, round(lst[j], D_PRES) - round(lst[j-1], D_PRES))
        if lc == 0:
            print("WARNING: X coordinates lost uniqueness due truncation")
        lst = sort(yl)
        lc = round(lst[1], D_PRES) - round(lst[0], D_PRES)
        for j in range(1,lst.size):
            lc = min(lc, round(lst[j], D_PRES) - round(lst[j-1], D_PRES))
        if lc == 0:
            print("WARNING: Y coordinates lost uniqueness due truncation")
    uvl = edges.axes[0].values      # directional edges
    lenl = edges["length"].values
    avg_x, avg_y = 0, 0
    for x in xl:
        avg_x += x
    for y in yl:
        avg_y += y
    avg_x = round(avg_x / xl.size, D_PRES)
    avg_y = round(avg_y / yl.size, D_PRES)
    for i in range(xl.size):
        xl[i] = round((xl[i] - avg_x) * 10**3, max(D_PRES - 3, 0))
    for i in range(yl.size):
        yl[i] = round((yl[i] - avg_y) * 10**3, max(D_PRES - 3, 0))
    UID_to_ind = {}
    ind_to_UID = []
    nodesl = []
    gc = 0
    for i in range(xl.size):
        UID_to_ind[uidl[i]] = gc
        ind_to_UID.append(uidl[i])
        nodesl.append((xl[i], yl[i]))
        gc += 1
    edgesl = []
    while gc > 0:
        edgesl.append([])
        gc -= 1
    for i in range(uvl.size):
        edgesl[UID_to_ind[uvl[i][0]]].append((UID_to_ind[uvl[i][1]], round(lenl[i], max(D_PRES - 3, 0))))
    print("Graph network decomposed!")
    return (nodesl, edgesl, UID_to_ind, ind_to_UID)


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
