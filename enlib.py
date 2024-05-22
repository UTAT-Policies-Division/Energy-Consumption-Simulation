from math import sqrt, acos, pi
import matplotlib.pyplot as plt

class EnergyHelper:
  """
  stores nodes and edges globally.
  """

  def __init__(self, nodes, edges, angle_tolerance, gen_plot_data=False):
    self.nodes = nodes
    self.edges = edges
    self.ang_tol = angle_tolerance
    self.half_pi = pi / 2
    self.three_half_pi = 3 * (pi / 2)
    self.two_pi = 2 * pi
    self.line_cover = None
    if gen_plot_data:
      self.line_cover = self.gen_network_line_cover()

  def get_displacement_vector(self, u, v):
    """
    get 2D displacement vector between two verticies.
    """
    ux, uy = self.nodes[u]
    vx, vy = self.nodes[v]
    return (vx - ux, vy - uy)
  
  def get_inner_product(self, d1, d2):
    """
    get 2D inner product of two vectors.
    """
    return (d1[0] * d2[0]) + (d1[1] * d2[1])

  def get_vector_norm(self, d):
    """
    get norm of 2D vector.
    """
    return sqrt((d[0] * d[0]) + (d[1] * d[1]))

  def get_turn_angle(self, u, v, w):
    """
    get turn angle turned in u -> v -> w.
    """
    dvu = self.get_displacement_vector(v, u)
    dvw = self.get_displacement_vector(v, w)
    return acos(self.get_inner_product(dvu, dvw) / 
                (self.get_vector_norm(dvu) * self.get_vector_norm(dvw)))

  def classify_turn_angle(self, u, v, w):
    """
    get classification of turn angle in u -> v -> w.
    """
    angle = self.get_turn_angle(u, v, w)
    if angle < self.ang_tol:
      return "U-turn"
    elif abs(self.half_pi - angle) < self.ang_tol:
      return "90*-turn"
    elif abs(pi - angle) < self.ang_tol:
      return "straight_road"
    return "ERR: could not classify turn"
  
  def get_edge_length(self, u, v):
    """
    get the length of the road connecting u and v,
    -1 if no road found.
    """
    for nbrs in self.edges[u]:
      if nbrs[0] == v:
        return nbrs[1]
    return -1
  
  def edge_exists(self, u, v):
    """
    get whether there exists an edge between u and v.
    based on the fact that if two vertices have distance 0,
    they must be the same vertex.
    """
    return self.get_edge_length(u, v) > 0
  
  def gen_edges_tracker(self):
    """
    generate edges visited tracker.
    """
    edt = []
    for k in range(len(self.edges)):
      # hold the valid last index.
      edt.append([len(self.edges[k]) - 1,[]])
      for _ in self.edges[k]:
        edt[k][1].append(0)
    return edt

  def _exists_neighbor(k, nbs):
    # skip all neighbors which have been visited.
    while nbs[0] >= 0:
      if nbs[1][nbs[0]] > 0:
        nbs[0] -= 1
      else:
        break
    # if the last index is not 0, then there exists
    # some unvisited neighbor!
    return nbs[0] >= 0

  def gen_network_line_cover(self):
    """
    returns tuple of x and y component
    of line segments covering the network.
    """
    print("Generating line segement cover for network...")
    segs_x, segs_y = [], []
    edt = self.gen_edges_tracker()
    N = len(self.edges)
    cnt = 0
    min_index = 0
    cr = 0
    c_x, c_y = -1, -1
    cl_x, cl_y = [], []
    while min_index < N:
      cr = min_index
      nbs = edt[cr]
      c_x, c_y = self.nodes[cr]
      cl_x, cl_y = [c_x], [c_y]
      if not EnergyHelper._exists_neighbor(cr, nbs):
        min_index += 1
      else:
        while EnergyHelper._exists_neighbor(cr, nbs):
          nbs[1][nbs[0]] = 1
          nb_uid = self.edges[cr][nbs[0]][0]
          nbs[0] -= 1
          cnt = 0
          for p in self.edges[nb_uid]:
            if p[0] == cr:
              edt[nb_uid][1][cnt] = 1
              break
            cnt+=1
          cr = nb_uid
          nbs = edt[cr]
          c_x, c_y = self.nodes[cr]
          cl_x.append(c_x)
          cl_y.append(c_y)
        segs_x.append(cl_x)
        segs_y.append(cl_y)
    print("Line segement cover generated!")
    return (segs_x, segs_y)

  def plot_network(self):
    """
    plot given graph network
    """
    x = []
    y = []
    for p in self.nodes:
      x.append(p[0])
      y.append(p[1])
    if self.line_cover is None:
      self.line_cover = self.gen_network_line_cover()
    llx, lly = self.line_cover
    for i in range(len(llx)):
      plt.plot(llx[i], lly[i], marker="")
    plt.scatter(x, y, c="red", s=2)
