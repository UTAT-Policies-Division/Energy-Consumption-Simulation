from math import sqrt, acos, pi, cos, exp, atan2
import matplotlib.pyplot as plt


half_pi = pi / 2
three_half_pi = 3 * (pi / 2)
two_pi = 2 * pi
RPM_coeff = two_pi / 60
tmp = 0
tmp1 = 0


def inner_product(d1, d2):
  """
  get 2D inner product of two vectors.
  """
  return (d1[0] * d2[0]) + (d1[1] * d2[1])

def vector_norm(d):
  """
  get norm of 2D vector.
  """
  return sqrt((d[0] * d[0]) + (d[1] * d[1]))

def RPM_to_omega(rpm):
  """
  RPM to theta / sec
  """
  return rpm * RPM_coeff

def omega_to_RPM(omega):
  """
  theta / sec to RPM
  """
  return omega / RPM_coeff

def c(r):
  """
  chord lengths for 1 <= r <= 11 inches, 
  result in mm.
  assumes safe use for efficiency.
  """
  if r <= 4:
    r -= 4
    tmp = r
    r *= r
    tmp1 = 0.652444*cos(3.14425*tmp) + 50.6977 - \
           1.20882*tmp + (1.58523 + 3.23691*tmp)*r
    r *= r
    return tmp1 - 0.208061*r*tmp
  elif r <= 7:
    r -= 4
    tmp = r
    r *= r
    return 0.114129*cos(2.41374*tmp) + 51.2251 - \
           0.253086*tmp - (1.00919 - 0.0548433*tmp)*r
  else:
    tmp = r
    r *= r
    return -1*exp(-143.87179093 + 13.3561*tmp) + \
           63.8221 - (0.55019 - 0.0178557*tmp)*r

def l(rho, V_T, phi, r):
  """
  r in inches.
  lift curve slope, 2D, taken 2 pi.
  beta = arctan(P / 2 pi r), for 22x8
  since fixed pitch.
  """
  return rho * pi * V_T * V_T * \
         c(r) * 0.001 * (atan2(8, two_pi*r) - phi)

def draw_function(s_x, e_x, f):
  """
  draw function f between s_x and e_x.
  """
  lx = [s_x]
  ly = [c(s_x)]
  for i in range(s_x*100, (e_x*100)+1, 1):
      lx.append(i/100)
      ly.append(f(i/100))
  plt.plot(lx, ly)


class EnergyHelper:
  """
  stores nodes and edges globally.
  """

  def __init__(self, nodes: list, edges: list, dedges: list, UID_to_ind, 
               ind_to_UID, angle_tolerance, gen_plot_data=False, demand=[]):
    self.nodes = nodes
    self.edges = edges
    self.dedges = dedges
    self.ang_tol = angle_tolerance
    self.line_cover = None
    self.line_cover_d = None
    if gen_plot_data:
      self.line_cover = self.gen_network_line_cover(self.edges)
      self.line_cover_d = self.gen_network_line_cover(self.dedges)
  
  def add_demand_node(self, index, weight):
    if index < 0 or index >= len(self.nodes):
      raise IndexError("Demand Index out of bounds.")
    elif weight < 0:
      raise ValueError("Demand Weight cannot be negative.")
    self.total_weight += weight
    if self.total_weight >= 4500:
      print("WARNING: Total demand weight exceeds 4500kg critical point.")
    self.demand.append((index, weight))

  def add_demand_list(self, lst):
    for i, w in lst:
      self.add_demand_node(i, w)

  def get_displacement_vector(self, u, v):
    """
    get 2D displacement vector between two verticies.
    """
    ux, uy = self.nodes[u]
    vx, vy = self.nodes[v]
    return (vx - ux, vy - uy)

  def get_turn_angle(self, u, v, w):
    """
    get turn angle turned in u -> v -> w.
    """
    dvu = self.get_displacement_vector(v, u)
    dvw = self.get_displacement_vector(v, w)
    return acos(inner_product(dvu, dvw) / 
                (vector_norm(dvu) * vector_norm(dvw)))
    return acos(inner_product(dvu, dvw) / 
                (vector_norm(dvu) * vector_norm(dvw)))

  def classify_turn_angle(self, u, v, w):
    """
    get classification of turn angle in u -> v -> w.
    """
    angle = self.get_turn_angle(u, v, w)
    if angle < self.ang_tol:
      return "U-turn"
    elif abs(half_pi - angle) < self.ang_tol:
    elif abs(half_pi - angle) < self.ang_tol:
      return "90*-turn"
    elif abs(pi - angle) < self.ang_tol:
      return "straight_road"
    return "ERR: could not classify turn"
  
  def get_edge_length_drive(self, u, v):
    """
    get the length of the road connecting u and v,
    -1 if no road found.
    """
    for nbrs in self.edges[u]:
      if nbrs[0] == v:
        return nbrs[1]
    return -1

  def get_edge_length_bike(self, u, v):
    """
    get the length of the road connecting u and v,
    -1 if no road found.
    """
    for nbrs in self.edges[u]:
      if nbrs[0] == v:
        return nbrs[1]
    for nbrs in self.dedges[u]:
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
    
    returns -1 is not found
             0 if in drive network
             1 if in bike newtork
    """
    for nbrs in self.edges[u]:
      if nbrs[0] == v:
        return 0
    for nbrs in self.dedges[u]:
      if nbrs[0] == v:
        return 1
    return -1

  def gen_edges_tracker(self, edge_data):
    """
    generate edges visited tracker.
    """
    edt = []
    for k in range(len(self.edges)):
    edt = []
    for k in range(len(edge_data)):
      # hold the valid last index.
      edt.append([len(self.edges[k]) - 1,[]])
      for _ in self.edges[k]:
      edt.append([len(edge_data[k]) - 1,[]])
      for _ in edge_data[k]:
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

  def gen_network_line_cover(self, edge_data):
    """
    returns tuple of x and y component
    of line segments covering the network.
    """
    print("Generating line segement cover for one network...")
    segs_x, segs_y = [], []
    edt = self.gen_edges_tracker()
    N = len(self.edges)
    cnt = 0
    edt = self.gen_edges_tracker(edge_data)
    N = len(edge_data)
    cnt = 0
    min_index = 0
    cr = 0
    cr = 0
    c_x, c_y = -1, -1
    cl_x, cl_y = [], []
    while min_index < N:
      cr = min_index
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
          nb_uid = edge_data[cr][nbs[0]][0]
          nbs[0] -= 1
          cnt = 0
          for p in edge_data[nb_uid]:
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
    print("Plotting network...\nGreen: drone only, Blue: all, Red: node, Crimson: demand node (larger).")
    got = [0 for _ in range(len(self.nodes))]
    nx, dx = [], []
    ny, dy = [], []
    for p in self.demand:
      got[p[0]] = 1
      dx.append(self.nodes[p[0]][0])
      dy.append(self.nodes[p[0]][1])
    for i in range(len(self.nodes)):
      if got[i] == 1:
        continue
      nx.append(self.nodes[i][0])
      ny.append(self.nodes[i][1])
    if self.line_cover is None:
      self.line_cover = self.gen_network_line_cover(self.edges)
    if self.line_cover_d is None:
      self.line_cover_d = self.gen_network_line_cover(self.dedges)
    plt.scatter(dx, dy, c="crimson", s=12)
    plt.scatter(nx, ny, c="red", s=4)
    llx, lly = self.line_cover
    for i in range(len(llx)):
      plt.plot(llx[i], lly[i], marker="")
    plt.scatter(x, y, c="red", s=2)

class EnergyFunction:
  """
  General BET (Blade Element Theory) with Coleman Inflow Model.

  rho: density of surrounding air
  V: forward speed

  S_ref: reference area, usually drone front area
  C_D_alph0: drag force coefficient when alpha_D = 0
    D_f = 0.5 * C_D_alph0 * S_ref * rho * V^2

  V_w_hd: wind speed counter V (head) component
  V_w_lt: wind speed lateral downwards component

  """
  def __init__(self, C_D_alph0, S_ref):
    self.C_D_alph0 = C_D_alph0
    self.S_ref = S_ref
  
  def D_f(self, rho, V):
    """
    Drag force.
    TODO: include effect of having a package.
    """
    return 0.5 * self.C_D_alph0 * self.S_ref * rho * V * V
