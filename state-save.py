import osmnx.simplification
import osmnx.distance
import networkx as nx
from tqdm import tqdm
from random import sample
import multiprocessing as mp
import pickle

RADIUS_SHIFT = 50
TAGS = [{'amenity': 'prison'},
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
         'power': ['generator', 'planet', 'substation'],
         'historic': ['fort', 'building', 'memorial'],
         'gnis:feature_id':	['2781516', '2083047', '2083660', '949654'],
         'wikidata': ['Q11259', 'Q110007', 'Q11277', 'Q2982151'],
         'tiger:cfcc': 'A41:A49:A63'},
        {'highway': ['motorway', 'trunk'],
         'building': ['firestation', 'hospital']},
        {'amenity': ['school', 'college', 'university'],
         'public_transport': 'station'},
        {"building": True}]

class PolicyData:
    def __init__(self):
        self.REGION_POLICY = {
            'NO_FLY_ZONES': {'set 1': [],
                             'set 2': [],
                             'set 3': [],
                             'set 4': [],
                             'set 5': []}
        }

def no_fly_zones_init(zone, policy_object, epsg):
    keys = ['set 1', 'set 2', 'set 3', 'set 4', 'set 5']
    for j in range(len(TAGS)):
        if TAGS[j]:
            geoms =  osmnx.project_gdf(osmnx.features_from_place(zone, TAGS[j]), epsg)['geometry']
            indices = list(range(len(geoms)))
            if j == 4:
               print("Special tag called.")
               indices = sample(indices, int(0.15 * len(indices)))
            for i in indices:
                center = geoms[i].centroid
                cx, cy = center.x, center.y
                lx, ly, ux, uy = geoms[i].bounds
                rad = (max(cx - lx, ux - cx) + max(cy - ly, uy - cy)) / 2
                for k in range(j, len(TAGS)):
                    policy_object.REGION_POLICY['NO_FLY_ZONES'][keys[k]].append((cx, cy, rad))

def remove_no_fly_zones(tgt_x_y_r, org_graph, q):
    q.put(len(tgt_x_y_r))
    for x, y, r in tgt_x_y_r:
        r += RADIUS_SHIFT
        # node, dst = osmnx.distance.nearest_nodes(org_graph, x, y, return_dist=True)
        # while dst < r:
        #     org_graph.remove_node(node)
        #     node, dst = osmnx.distance.nearest_nodes(org_graph, x, y, return_dist=True)
        edge, dst = osmnx.distance.nearest_edges(org_graph, x, y, return_dist=True)
        while dst < r:
            org_graph.remove_edge(*edge)
            edge, dst = osmnx.distance.nearest_edges(org_graph, x, y, return_dist=True)
        q.put(0)

simplification_tolerance=1
epsg = "EPSG:32118"
place_name = "Manhattan"
boundary_buffer_length = 500
OSM_CRS_EPSG = "EPSG:4326"

class Storage:
  def __init__(self):
    self.xl = None
    self.yl = None
    self.uidl = None
    self.ulb = None   # directional edges
    self.vlb = None
    self.lenlb = None
    self.uld = None
    self.vld = None
    self.lenld = None

  def save(self, filename='network_data.pkl'):
    print("Saving Storage object...")
    output = open(filename, 'wb')
    pickle.dump(self, output, 2)
    output.close()
    print("Storage object saved!")
  
  def load(filename='network_data.pkl'):
    print("Loading Storage object...")
    input = open(filename, 'rb')
    sobj = pickle.load(input)
    input.close()
    print("Storage object loaded!")
    obj = Storage()
    obj.xl = sobj.xl
    obj.yl = sobj.yl
    obj.uidl = sobj.uidl
    obj.ulb = sobj.ulb   # directional edges
    obj.vlb = sobj.vlb
    obj.lenlb = sobj.lenlb
    obj.uld = sobj.uld
    obj.vld = sobj.vld
    obj.lenld = sobj.lenld
    return obj

def _osmnx_worker(no_fly_zones, index, q):
  place_polygon = osmnx.geocode_to_gdf(place_name).to_crs(epsg)
  place_polygon["geometry"] = place_polygon.buffer(boundary_buffer_length).to_crs(OSM_CRS_EPSG)
  graphB = osmnx.project_graph(osmnx.graph_from_polygon(place_polygon.at[0, "geometry"], network_type="bike"), epsg)
  graphD = osmnx.project_graph(osmnx.graph_from_polygon(place_polygon.at[0, "geometry"], network_type="drive"), epsg)
  if simplification_tolerance > 0:
      graphB = osmnx.simplification.consolidate_intersections(graphB, tolerance=simplification_tolerance)
      graphD = osmnx.simplification.consolidate_intersections(graphD, tolerance=simplification_tolerance)
  merged_graph = nx.compose(graphD, graphB)
  edge_data = {
      e: min(graphB.edges[e]["length"], graphD.edges[e]["length"]) for e in graphB.edges & graphD.edges
  }
  nx.set_edge_attributes(merged_graph, edge_data, "length")
  graphB = merged_graph
  q.put(0)
  remove_no_fly_zones(no_fly_zones, graphB, q)
  nodesB, edgesB = osmnx.graph_to_gdfs(graphB)
  edgesD = osmnx.graph_to_gdfs(graphD, nodes=False, edges=True)
  obj = Storage()
  obj.xl = list(nodesB["x"].values)
  obj.yl = list(nodesB["y"].values)
  obj.uidl = list(nodesB["osmid_original"].values)
  obj.ulb = edgesB["u_original"].values   # directional edges
  obj.vlb = edgesB["v_original"].values
  obj.lenlb = edgesB["length"].values
  obj.uld = edgesD["u_original"].values
  obj.vld = edgesD["v_original"].values
  obj.lenld = edgesD["length"].values
  obj.save('set-' + str(index) + ".pkl")
  q.put(0)

def listener(q):
    pbar = tqdm(total = 1 * 2)
    while q.empty():
        continue
    upd = q.get()
    while upd != None:
        if upd > 0:
            pbar.total = pbar.total + upd
            pbar.refresh()
        else:
            pbar.update()
        while q.empty():
            continue
        upd = q.get()
    pbar.close()

def download_data_parallel(policy_object):
  q = mp.Queue()
  proc = mp.Process(target=listener, args=(q,))
  proc.start()
  workers = [mp.Process(target=_osmnx_worker, 
                        args=(policy_object.REGION_POLICY['NO_FLY_ZONES']["set {}".format(i+1)], 
                              i+1, q)) for i in range(4, 5)]
  for worker in workers:
    worker.start()
  for worker in workers:
    worker.join()
  q.put(None)
  proc.join()

if __name__ == '__main__':
    print("Getting data from OSM...")
    policy_object = PolicyData()
    no_fly_zones_init(["Manhattan, United States"], policy_object, epsg)
    download_data_parallel(policy_object)
    print("Saved all data from OSM!")
