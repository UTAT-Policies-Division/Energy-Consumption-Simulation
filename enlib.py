from math import sqrt, acos, atan2, pi, \
                 sin, cos, tan, exp, \
                 floor, ceil
from random import randint, sample
import matplotlib.pyplot as plt
import pickle
import multiprocessing as mp
from os import cpu_count
from tqdm import tqdm
import heapq

half_pi = pi / 2
three_half_pi = 3 * (pi / 2)
two_pi = 2 * pi
RPM_coeff = two_pi / 60
meter_coeff = 39.3700787
mps_coeff = 3.6
rho_air_std = 1.204 # kg / m^3
kgs_coeff = 9.81
deg_coeff = pi / 180
PROP_START, PROP_END = 1, 11  # inches
DRAW_PREC = 100 # power of 10, larger => more precise
AREA = pi * (11 / meter_coeff)**2
NEWT_PREC = 10**(-5)
BATTERY_RESERVE_MARGIN = 0.2
BATTERY_CAPACITY = 17.0 * 48 * 3600  # J
MAX_BATTERY_USE = BATTERY_CAPACITY * (1 - BATTERY_RESERVE_MARGIN)
MAX_BATTERY_USE_HALF = MAX_BATTERY_USE / 2
C_D_ALPHA0, S_REF, DRONE_GROUND_SPEED = -1, -1, -1
CHORD, BETA, SINPSI, COSPSI = -1, -1, -1, -1
MAX_TRUCK_SPEED, BASE_TRUCK_SPEED, TRUCK_CITY_MPG = -1, -1, -1
BASE_TEMP, TEMP_FLUC_COEFF, REL_HUMIDITY = -1, -1, -1
QUAD_A, QUAD_B, QUAD_C = -1, -1, -1
WEIGHTS = [0, 0.25, 0.5, 1, 1.5, 2]

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

def meters_to_inches(meter):
  return meter * meter_coeff

def inches_to_meters(inch):
  return inch / meter_coeff

def mps_to_kph(mps):
  return mps * mps_coeff

def kph_to_mps(kph):
  return kph / mps_coeff

def kgs_to_W(kgs):
  return kgs * kgs_coeff

def deg_to_rad(deg):
  return deg * deg_coeff

def rad_to_deg(rad):
  return rad / deg_coeff

def c(r):   # correct, but prefer not used
  """
  chord lengths for 1 <= r <= 11 inches, 
  result in mm, for 22x8 propeller
  assumes safe use for efficiency.
  """
  tmp, tmp1 = 0, 0
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

def l(rho, V_T_sq, phi, r):   # correct, but prefer not used
  """
  r in inches.
  lift curve slope, 2D, taken 5.7 .
  beta = arctan(P / 2 pi r), for 22x8
  since fixed pitch.
  """
  return rho * 2.85 * V_T_sq * c(r) * \
         0.001 * (atan2(8, two_pi*r) - phi)

def d(rho, V_T_sq, r):   # correct, but prefer not used
  """
  r in inches.
  c_d is 2D drag coefficient, taken
  to be 0.045 for airfoil.
  """
  return rho * 0.0000225 * V_T_sq * c(r)

def V_T_sq(V_x, psi, omega, r, V_c, v):   # correct, but prefer not used
  """
  r in inches.
  norm of thrust velocity square.
  """
  return (V_x * sin(psi) + omega * \
          inches_to_meters(r))**2 + (V_c + v)**2

def phi(V_x, psi, omega, r, V_c, v):   # correct, but prefer not used
  """
  r in inches.
  angle of thrust velocity.
  """
  return atan2(V_c + v, V_x * sin(psi) + \
               omega * inches_to_meters(r))

def v(v0, x, V_x, V_c, psi):   # correct, but prefer not used
  """
  x is normalized radius
  """
  return v0 * (1 + x * tan(atan2(V_x, v0 + V_c) / 2) * cos(psi))

def v0(rho, T, V_x_sq, V_c):   # correct, but prefer not used
  C = T / (2 * rho * AREA)
  C *= C
  k_v = 2
  VAR1 = k_v + V_c
  fv = ((k_v * k_v * (VAR1 * VAR1 + V_x_sq)) - C)
  dv = fv / (2 * k_v * ((VAR1 * (VAR1 + k_v)) + V_x_sq))
  while abs(fv) > NEWT_PREC:
    k_v -= dv
    VAR1 = k_v + V_c
    fv = ((k_v * k_v * (VAR1 * VAR1 + V_x_sq)) - C)
    dv = fv / (2 * k_v * ((VAR1 * (VAR1 + k_v)) + V_x_sq))
  return k_v

def f_z(rho, V_T_sq, phi, r):   # correct, but prefer not used
  """
  r in inches.
  force in +z direction.
  """
  return l(rho, V_T_sq, phi, r) * cos(phi) - \
         d(rho, V_T_sq, r) * sin(phi)

def f_x(rho, V_T_sq, phi, r):   # correct, but prefer not used
  """
  r in inches.
  force in +x direction.
  """
  return l(rho, V_T_sq, phi, r) * sin(phi) + \
         d(rho, V_T_sq, r) * cos(phi)

def init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
                 base_temperature=20, temp_flucts_coeff=3, relative_humidity=0.7,
                 drone_speed=18):
  global C_D_ALPHA0, S_REF, CHORD, BETA, SINPSI, COSPSI, DRONE_GROUND_SPEED, \
         MAX_TRUCK_SPEED, BASE_TRUCK_SPEED, TRUCK_CITY_MPG, BASE_TEMP, \
         TEMP_FLUC_COEFF, REL_HUMIDITY, QUAD_A, QUAD_B, QUAD_C
  CHORD = []  # holds in millimeters
  BETA = []
  _r = 1.0
  d = 0.1
  for _ in range(0, 100):
    CHORD.append(c(_r))
    BETA.append(atan2(8, 2 * pi * _r))
    _r += d
  SINPSI = []
  COSPSI = []
  psi = 0
  d = 2 * pi / 100
  for _ in range(0, 100):
    SINPSI.append(sin(psi))
    COSPSI.append(cos(psi))
    psi += d
  C_D_ALPHA0 = 0.5
  S_REF = 0.05
  DRONE_GROUND_SPEED = drone_speed
  MAX_TRUCK_SPEED = max_truck_speed
  BASE_TRUCK_SPEED = base_truck_speed
  TRUCK_CITY_MPG = truck_city_mpg
  BASE_TEMP = base_temperature
  TEMP_FLUC_COEFF = temp_flucts_coeff / 2
  REL_HUMIDITY= relative_humidity
  QUAD_C = 74736280 / truck_city_mpg
  QUAD_B = -sqrt(74736280 - QUAD_C)
  QUAD_A = QUAD_B / -24.5872

def copy_globals(drone_speed):
  global C_D_ALPHA0, S_REF, DRONE_GROUND_SPEED, \
         CHORD, BETA, SINPSI, COSPSI
  CHORD = []  # holds in millimeters
  BETA = []
  _r = 1.0
  d = 0.1
  for _ in range(0, 100):
    CHORD.append(c(_r))
    BETA.append(atan2(8, 2 * pi * _r))
    _r += d
  SINPSI = []
  COSPSI = []
  psi = 0
  d = 2 * pi / 100
  for _ in range(0, 100):
    SINPSI.append(sin(psi))
    COSPSI.append(cos(psi))
    psi += d
  C_D_ALPHA0 = 0.5
  S_REF = 0.05
  DRONE_GROUND_SPEED = drone_speed

def TH_BET(rho, v0, Vx, Vc, omega, CHORD, BETA, SINPSI, COSPSI):   # correct, but prefer not used
  resT, resH = 0, 0
  T, H, x = 0, 0, 0
  dr_SI = 0.00254
  dx = 0.009090909091
  dv = 0
  i, j = 0
  vCOEFF_INIT = tan(atan2(Vx, v0 + Vc) / 2)
  vCOEFF = 0
  Vc_v_sum1, Vc_v_sum2 = 0, 0
  omega_r = 0
  d_omega_r = omega * dr_SI
  SUM = 0
  phi = 0
  VTsq_c = 0
  l, d = 0, 0
  lCOEFF = rho * 2.85 * 0.001
  dCOEFF = rho * 0.0000225
  cosphi, sinphi = 0, 0
  HCOEFF = 0
  while i < 100:
    T = 0
    H = 0
    x = 0.09090909091
    j = 0
    omega_r = omega * 0.0254
    vCOEFF = vCOEFF_INIT * COSPSI[i]
    dv = v0 * dx * vCOEFF
    SUM = x * vCOEFF
    Vc_v_sum1 = Vc + v0 * (1 + SUM)
    Vc_v_sum2 = Vc + v0 * (1 - SUM)
    Vx_sinpsi = Vx * SINPSI[i]
    HCOEFF = SINPSI[i] * dr_SI
    while j < 100:
      SUM = omega_r + Vx_sinpsi
      if SUM > 0:
        phi = atan2(Vc_v_sum1, SUM)
        VTsq_c = (Vc_v_sum1 * Vc_v_sum1 + SUM * SUM) * CHORD[j]
        l = lCOEFF * min(0.2, max(BETA[j] - phi, -0.2)) * VTsq_c
        d = dCOEFF * VTsq_c
        cosphi, sinphi = cos(phi), sin(phi)
        T += (l*cosphi - d*sinphi) * dr_SI
        H += (l*sinphi + d*cosphi) * HCOEFF
      SUM = omega_r - Vx_sinpsi
      if SUM > 0:
        phi = atan2(Vc_v_sum2, SUM)
        VTsq_c = (Vc_v_sum2 * Vc_v_sum2 + SUM * SUM) * CHORD[j]
        l = lCOEFF * min(0.2, max(BETA[j] - phi, -0.2)) * VTsq_c
        d = dCOEFF * VTsq_c
        cosphi, sinphi = cos(phi), sin(phi)
        T += (l*cosphi - d*sinphi) * dr_SI
        H -= (l*sinphi + d*cosphi) * HCOEFF
      x += dx
      Vc_v_sum1 += dv
      Vc_v_sum2 -= dv
      omega_r += d_omega_r
      j += 1
    resT += T
    resH += H
    i += 1
  return (resT * 0.01, resH * 0.01)

def draw_function(s_x, e_x, dx, f):
  """
  draw function f between s_x and e_x with step dx.
  """
  lx, ly = [], []
  tmp = 0
  for i in range(floor(s_x/dx), ceil(e_x/dx)):
      tmp = i * dx
      lx.append(tmp)
      ly.append(f(tmp))
  plt.plot(lx, ly)

def draw_functions(s_x, e_x, dx, f, p_s, p_e, dp):
  """
  draw function f between s_x and e_x with step dx,
  varying 2nd paramter from p_s to p_e with step d.
  """
  lx, ly = [], []
  R, G, B = 0, 1, 0
  num = floor((p_e - p_s) / dp)
  p = p_s
  d_R, d_G, d_B = 0, -0.98/ceil(num/2), 0.98/ceil(num/2)
  tmp = 0
  for _ in range(ceil(num/2)):
    for i in range(floor(s_x/dx), ceil(e_x/dx)):
      tmp = i * dx
      lx.append(tmp)
      ly.append(f(tmp, p))
    plt.plot(lx, ly, color=(R, G, B), label=str(p))
    R += d_R
    G += d_G
    B += d_B
    lx.clear()
    ly.clear()
    p += dp
  R, G, B = 0, 0, 1
  d_R, d_G, d_B = 0.98/(num-ceil(num/2)), 0, -0.98/(num-ceil(num/2))
  for _ in range(ceil(num/2), num):
    for i in range(floor(s_x/dx), ceil(e_x/dx)):
      tmp = i * dx
      lx.append(tmp)
      ly.append(f(tmp, p))
    plt.plot(lx, ly, color=(R, G, B), label=str(p))
    R += d_R
    G += d_G
    B += d_B
    lx.clear()
    ly.clear()
    p += dp

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
    self.UID_to_ind = UID_to_ind
    self.ind_to_UID = ind_to_UID
    self.demand = demand
    self.total_weight = 0
    for dat in demand:
      self.total_weight += dat[1]
    self.sp_poss = None
    self.n_pherm = None
    self.llep_d = None
    self.lep_t = None
    self.let_t = None
    self.line_cover = None
    self.line_cover_d = None
    if gen_plot_data:
      self.line_cover = self.gen_network_line_cover(self.edges)
      self.line_cover_d = self.gen_network_line_cover(self.dedges)
  
  def enforce_graph_connections(self):
    count, ind = 0, 0
    found, tgt_edge, e = False, None, None
    q, to_add = [], []
    got = [0 for _ in range(len(self.nodes))]
    print("Finding nodes which are purely source / drain...")
    for n in range(len(self.nodes)):
      if len(self.edges[n]) == 0:
        continue
      found = False
      for e in self.edges[n]:
        for re in self.edges[e[0]]:
          if n == re[0]:
            found = True
            break
      if not found:
        count += 1
        self.edges[n] = []
    print("Complete! About", round(count * 100 / len(self.nodes), 2), "percent of all nodes were unreachable via truck.")
    print("Finding edges which need to be added to ensure well-connectedness of drive network...")
    count = 0
    for n in range(len(self.nodes)):
      for i in range(len(self.edges[n])):
        e = self.edges[n][i]
        if len(self.edges[e[0]]) == 0:
          continue
        for j in range(len(got)):
          got[j] = 0
        q.append(e[0])
        got[e[0]] = 1
        while len(q) > 0:
          ind = q.pop()
          for _e in self.edges[ind]:
            if got[_e[0]] == 0 and len(self.edges[_e[0]]) > 0:
              got[_e[0]] = 1
              q.append(_e[0])
        for _e in self.edges[n]:
          if got[_e[0]] == 0 and len(self.edges[_e[0]]) > 0:
            to_add.append(i)
            break
      count += len(to_add)
      while len(to_add) > 0:
        ind = to_add.pop()
        tgt_edge = self.edges[n][ind]
        self.edges[tgt_edge[0]].append((n, tgt_edge[1], tgt_edge[2], tgt_edge[3]))
    print(count, "road connections were made both-way!")
    # ind = 260
    # for i in range(len(got)):
    #   got[i] = 0
    # while len(self.edges[ind]) == 0:
    #   print("UHOH")
    #   ind = (ind + 1) % len(self.nodes)
    # NUM = len(self.nodes)
    # for i in range(len(self.nodes)):
    #   if len(self.edges[i]) == 0:
    #     got[i] = 1
    #     NUM -= 1
    # got[ind] = 1
    # q.append(ind)
    # lst = [ind]
    # num = 1
    # while len(q) > 0:
    #   ind = q.pop()
    #   for e in self.edges[ind]:
    #     if got[e[0]] == 0 and len(self.edges[e[0]]) > 0:
    #       got[e[0]] = 1
    #       num += 1
    #       lst.append(e[0])
    #       q.append(e[0])
    # print(NUM - num)
    # lst = []
    # for i in range(len(got)):
    #   if got[i] == 0:
    #     print(i)
    #     lst.append(i)
    #     for e in self.edges[i]:
    #       # for re in self.edges[e[0]]:
    #       #   if i == re[0]:
    #       #     return -1
    #       if len(self.edges[e[0]]) > 0:
    #         print("----",e[0])
    #         lst.append(e[0])
    # print([e[0] for e in self.edges[267]])

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

  def classify_turn_angle(self, u, v, w):
    """
    get classification of turn angle in u -> v -> w.
    """
    angle = self.get_turn_angle(u, v, w)
    if angle < self.ang_tol:
      return "U-turn"
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
    for k in range(len(edge_data)):
      # hold the valid last index.
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
    segs_x, segs_y, segs_n = [], [], []
    edt = self.gen_edges_tracker(edge_data)
    N = len(edge_data)
    cnt = 0
    min_index = 0
    cr = 0
    c_x, c_y = -1, -1
    cl_x, cl_y, cl_n = None, None, None
    while min_index < N:
      cr = min_index
      nbs = edt[cr]
      c_x, c_y = self.nodes[cr]
      cl_x, cl_y, cl_n = [c_x], [c_y], [cr]
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
          cl_n.append(cr)
        segs_x.append(cl_x)
        segs_y.append(cl_y)
        segs_n.append(cl_n)
    print("Line segement cover generated!")
    return (segs_x, segs_y, segs_n)

  def plot_edge_phermones(self, edge_map, color, llx, lly, lln):
    x, y, dx, dy, cx, cy = 0, 0, 0, 0, 0, 0
    nphm, max_pherm, ln = -1, -1, None
    if isinstance(edge_map, dict):
      for k1 in edge_map:
        for k2 in edge_map[k1]:
          max_pherm = max(max_pherm, edge_map[k1][k2])
      for i in range(len(llx)):
        ln = lln[i]
        for j in range(1, len(llx[i])):
          if ln[j-1] in edge_map and ln[j] in edge_map[ln[j-1]]:
            x = llx[i][j-1]
            y = lly[i][j-1]
            dx = llx[i][j] - x
            dy = lly[i][j] - y
            cx = min(max(0.05 * dx, -0.05), 0.05)
            cy = min(max(0.05 * dy, -0.05), 0.05)
            x += cx
            y += cy
            dx -= cx
            dy -= cy
            nphm = edge_map[ln[j - 1]][ln[j]] / max_pherm
            plt.arrow(x, y, dx, dy, ec=color, fc=color,
                      length_includes_head=True, 
                      head_width=8*nphm, alpha=nphm)
          if ln[j] in edge_map and ln[j-1] in edge_map[ln[j]]:
            x = llx[i][j]
            y = lly[i][j]
            dx = llx[i][j - 1] - x
            dy = lly[i][j - 1] - y
            cx = min(max(0.05 * dx, -0.05), 0.05)
            cy = min(max(0.05 * dy, -0.05), 0.05)
            x += cx
            y += cy
            dx -= cx
            dy -= cy
            nphm = edge_map[ln[j]][ln[j - 1]] / max_pherm
            plt.arrow(x, y, dx, dy, ec=color, fc=color,
                      length_includes_head=True,
                      head_width=8*nphm, alpha=nphm)
    else:
      for i in range(len(self.nodes)):
        for j in range(len(self.nodes)):
          max_pherm = max(max_pherm, edge_map[i][j])
      for i in range(len(llx)):
        ln = lln[i]
        for j in range(1, len(llx[i])):
          nphm = edge_map[ln[j - 1]][ln[j]]
          if nphm > 1:
            x = llx[i][j-1]
            y = lly[i][j-1]
            dx = llx[i][j] - x
            dy = lly[i][j] - y
            cx = min(max(0.05 * dx, -0.05), 0.05)
            cy = min(max(0.05 * dy, -0.05), 0.05)
            x += cx
            y += cy
            dx -= cx
            dy -= cy
            nphm /= max_pherm
            plt.arrow(x, y, dx, dy, ec=color, fc=color,
                      length_includes_head=True, 
                      head_width=8*nphm, alpha=nphm)
          nphm = edge_map[ln[j]][ln[j - 1]]
          if nphm > 1:
            x = llx[i][j]
            y = lly[i][j]
            dx = llx[i][j - 1] - x
            dy = lly[i][j - 1] - y
            cx = min(max(0.05 * dx, -0.05), 0.05)
            cy = min(max(0.05 * dy, -0.05), 0.05)
            x += cx
            y += cy
            dx -= cx
            dy -= cy
            nphm /= max_pherm
            plt.arrow(x, y, dx, dy, ec=color, fc=color,
                      length_includes_head=True,
                      head_width=8*nphm, alpha=nphm)

  def plot_network(self, show_drone_only_nodes, show_drone_only_edges, show_demand_nodes, 
                   show_demand_paths, show_for_all_edges, enable_phermone_alpha, spec_ind=[], spec_path=[]):
    """
    plot given graph network
    """
    print("Plotting network...")
    print("Green: paths only for drone.")
    print("Blue: paths for both drone and truck.")
    print("Red [+ linear gradient with White]: truck reachable node [+ redness indicates phermone level].")
    print("Light Gray [+ linear gradient with White]: drone reachable only node.")
    print("Magenta: demand node (larger).")
    got = [0 for _ in range(len(self.nodes))]
    nx, ny, nc = [], [], []
    dx, dy = [], []
    max_pherm = 0
    for p in self.demand:
      got[p[0]] = 1
      dx.append(self.nodes[p[0]][0])
      dy.append(self.nodes[p[0]][1])
    if self.sp_pherm is not None:
      max_pherm = max(p for p in self.sp_pherm)
      base_c = 0.01 * max_pherm
      comp = 0
      if show_drone_only_nodes:
        for i in range(len(self.nodes)):
          if got[i] == 1:
            continue
          nx.append(self.nodes[i][0])
          ny.append(self.nodes[i][1])
          if len(self.edges[i]) > 0:
            comp = 0.9 - min(1.75 * ((1000 * self.sp_pherm[i] + base_c) / max_pherm), 0.9)
            nc.append((1, comp, comp))
          else:
            nc.append((0.7, 0.7, 0.7))
      else:
        for i in range(len(self.nodes)):
          if got[i] == 1 or len(self.edges[i]) == 0:
            continue
          nx.append(self.nodes[i][0])
          ny.append(self.nodes[i][1])
          comp = 0.9 - min(1.75 * ((1000 * self.sp_pherm[i] + base_c) / max_pherm), 0.9)
          nc.append((1, comp, comp))
    else:
      if show_drone_only_nodes:
        for i in range(len(self.nodes)):
          if got[i] == 1:
            continue
          nx.append(self.nodes[i][0])
          ny.append(self.nodes[i][1])
          if len(self.edges[i]) > 0:
            nc.append((1, 0, 0))
          else:
            nc.append((0.7, 0.7, 0.7))
      else:
        for i in range(len(self.nodes)):
          if got[i] == 1 or len(self.edges[i]) == 0:
            continue
          nx.append(self.nodes[i][0])
          ny.append(self.nodes[i][1])
          nc.append((1, 0, 0))
    if self.line_cover is None and show_for_all_edges:
      self.line_cover = self.gen_network_line_cover(self.edges)
    if self.line_cover_d is None and show_drone_only_edges:
      self.line_cover_d = self.gen_network_line_cover(self.dedges)
    if show_demand_nodes:
      plt.scatter(dx, dy, c="magenta", s=15)
    plt.scatter(x=nx, y=ny, color=nc, s=8)
    if show_for_all_edges:
      llx, lly, lln = self.line_cover
      if enable_phermone_alpha:
        self.plot_edge_phermones(self.dt_pherm, "mediumblue", llx, lly, lln)
      if show_demand_paths:
        self.plot_edge_phermones(self.t_pherm, "black", llx, lly, lln)
      if not (enable_phermone_alpha or show_demand_paths):
        for i in range(len(llx)):
          plt.plot(llx[i], lly[i], marker="", c="mediumblue", alpha=0.4)
    if show_drone_only_edges:
      llx, lly, lln = self.line_cover_d
      if enable_phermone_alpha:
        self.plot_edge_phermones(self.do_pherm, "limegreen", llx, lly, lln)
      else:
        for i in range(len(llx)):
          plt.plot(llx[i], lly[i], marker="", c="limegreen", alpha=0.4)
    for i in spec_ind:
      plt.scatter(x=self.nodes[i][0], y=self.nodes[i][1], c='black', s=15)
    path_x, path_y = None, None
    for path in spec_path:
      path_x, path_y = [], []
      for i in path:
        path_x.append(self.nodes[i][0])
        path_y.append(self.nodes[i][1])
      plt.plot(path_x, path_y, marker="", c="black", alpha=0.1)
    print("Plotted network!")

  def append_random_demand(self, num, cluster_num = 0, cluster_jump = 0, 
                           drone_only_possible_component=0.7, num_allocs=3):
    """
    Demand generated ALWAYS corresponds
    to nodes reachable by the truck.

    drone_only_possible_component is
    responsible for allocating pure
    drone only possible deliveries
    """
    print("Generating random demand...")
    N = len(self.nodes)
    LIM = N * N
    a, w_a = 0, 0
    b, w_b = N - 1, len(WEIGHTS) - 1
    got = [0 for _ in range(N)]
    for dem,_ in self.demand:
      got[dem] = 1
    n = 0
    if cluster_num <= 0:
      while num > 0:
        n = randint(a, b)
        while (got[n] > 0) or (len(self.edges[n]) == 0):
          n = (n + 1) % N
          if LIM == 1:
            self.demand = []
            self.total_weight = 0
            print("ERROR: could not find suitable node for demand.")
            return
          else:
            LIM -= 1
        got[n] = 1
        self.demand.append(n)
        num -= 1
    else:
      # cluster_jump = 0 implies immediate neighbors
      threshold = 1 + int(floor(num / cluster_num) * 0.1)
      to_add = 0
      q = [-1 for _ in range(N)]
      end = -1
      lv = -1
      for cluster_rem in range(cluster_num, 0, -1):
        n = randint(a, b)
        while (got[n] > 0) or (len(self.edges[n]) == 0):
          n = (n + 1) % N
          if LIM == 1:
            self.demand = []
            self.total_weight = 0
            print("ERROR: could not find suitable node for demand.")
            return
          else:
            LIM -= 1
        end = 0
        q[end] = (n, 0)
        to_add = max(1, ceil(num / cluster_rem))
        to_swap = -1
        tmp = -1
        while end >= 0:
          n, lv = q[end]
          end -= 1
          if lv == 0:
            got[n] = 1   # targeted & visited
            self.demand.append(n)
            num -= 1
            if to_add == 1:
              break
            else:
              for j in range(min(len(self.edges[n]), to_add + 1)):
                tmp = self.edges[n][j][0]
                if got[tmp] == 0:
                  got[tmp] = 3   # in queue
                  to_swap = randint(0, max(0, end))
                  end += 1
                  q[end] = (tmp, cluster_jump)
                  tmp = q[to_swap]
                  q[to_swap] = q[end]
                  q[end] = tmp
              to_add -= 1
          else:
            got[n] = 2   # not targted
            lv -= 1
            for j in range(min(len(self.edges[n]), to_add + 1)):
              tmp = self.edges[n][j][0]
              if got[tmp] == 0:
                got[tmp] = 3   # in queue
                to_swap = randint(0, max(0, end))
                end += 1
                q[end] = (tmp, lv)
                tmp = q[to_swap]
                q[to_swap] = q[end]
                q[end] = tmp
        if to_add > threshold:
          print("WARNING: cluster generation suppressed in area.")
    w_total, w = 0, 0
    pure_drone = sample([i for i in range(len(self.demand))], ceil(drone_only_possible_component * len(self.demand)))
    for i in range(len(self.demand)):
      w = 0
      if i in pure_drone:
        w = WEIGHTS[randint(w_a, w_b)]
      else:
        w = 0
        for _ in range(num_allocs):
          w += WEIGHTS[randint(w_a, w_b)]
      self.demand[i] = (self.demand[i], w)
      w_total += w
    self.total_weight = w_total
    if self.total_weight >= 4500:
      print("WARNING: Total demand weight exceeds 4500kg critical point.")
    print("Random demand generated!")

  def save(self, filename='network_data.pkl'):
    print("Saving Energy Helper object...")
    output = open(filename, 'wb')
    pickle.dump(self, output, 2)
    output.close()
    print("Energy Helper object saved!")
  
  def load(filename='network_data.pkl'):
    print("Loading Energy Helper object...")
    input = open(filename, 'rb')
    obj = pickle.load(input)
    input.close()
    print("Energy Helper object loaded!")
    ehobj = EnergyHelper(obj.nodes,
                         obj.edges,
                         obj.dedges,
                         obj.UID_to_ind,
                         obj.ind_to_UID,
                         obj.ang_tol,
                         False,
                         obj.demand)
    ehobj.line_cover = obj.line_cover
    ehobj.line_cover_d = obj.line_cover_d
    ehobj.sp_poss = obj.sp_poss
    ehobj.n_pherm = obj.n_pherm
    ehobj.llep_d = obj.llep_d
    ehobj.lep_t = obj.lep_t
    ehobj.let_t = obj.let_t
    if (obj.total_weight - ehobj.total_weight) > 0.0001:
      print("WARNING: total weight not consistent between actual demand and storage.")
    return ehobj

  def get_local_node_index(self, original_osmid):
    return self.UID_to_ind[original_osmid]
  
  def get_node_osmid(self, local_index):
    return self.ind_to_UID[local_index]
  
  def get_close_node(self, tx, ty, delta_tol = 10, drive_only=True):
    if drive_only:
      for i in range(len(self.nodes)):
        if len(self.edges[i]) == 0:
          continue
        if abs(tx - self.nodes[i][0]) < delta_tol:
          if abs(ty - self.nodes[i][1]) < delta_tol:
            return i
    else:
      for i in range(len(self.nodes)):
        if abs(tx - self.nodes[i][0]) < delta_tol:
          if abs(ty - self.nodes[i][1]) < delta_tol:
            return i

  def get_top_right_node(self):
    max_x, max_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] > max_x and self.nodes[i][1] > max_y:
        max_x = self.nodes[i][0]
        max_y = self.nodes[i][1]
        ind = i
    return ind
  
  def get_top_left_node(self):
    max_x, min_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] > max_x and self.nodes[i][1] < min_y:
        max_x = self.nodes[i][0]
        min_y = self.nodes[i][1]
        ind = i
    return ind
  
  def get_bottom_right_node(self):
    min_x, max_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] < min_x and self.nodes[i][1] > max_y:
        min_x = self.nodes[i][0]
        max_y = self.nodes[i][1]
        ind = i
    return ind
  
  def get_bottom_left_node(self):
    min_x, min_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] < min_x and self.nodes[i][1] < min_y:
        min_x = self.nodes[i][0]
        min_y = self.nodes[i][1]
        ind = i
    return ind

  def remove_phermones(self, R, dem_ind):
    R_HALF = (R / 2)  
    MAX_DST = 0.99 * R   # trying to ensure no negative.
    SP_PHERM_COEFF = 0.1
    T_PATH_BASE = 0.075
    T_PATH_COEFF = 0.1 - T_PATH_BASE
    T_PATH_EXPLR = 0.04
    DT_PATH_BASE = 0.025
    DT_PATH_COEFF = 0.1 - DT_PATH_BASE
    DO_PATH_BASE = 0.05
    DO_PATH_COEFF = 0.125 - DO_PATH_BASE
    nodes = self.nodes
    edges = self.edges
    dedges = self.dedges
    demand = self.demand
    got = [0 for _ in range(len(nodes))]
    q_ind, q_len = [], []
    heapq.heapify(q_ind)
    eng, ind, tent_eng = -1, -1, -1
    to_add, curr, w_i = None, -1, -1
    max_weight = max(w for _, w in demand)
    # Removing phermones from paths
    # leading to [dem_ind] demand.
    w_i = T_PATH_BASE + T_PATH_COEFF * (demand[dem_ind][1] / max_weight)
    for j in range(0, dem_ind):
      to_add = self.lep_t[dem_ind][demand[j][0]]
      curr = 1
      while curr < len(to_add):
        self.t_pherm[to_add[curr - 1]][to_add[curr]] -= min(w_i, self.t_pherm[to_add[curr - 1]][to_add[curr]])
        curr += 1
      self.t_pherm[to_add[curr - 1]][demand[dem_ind][0]] -= min(w_i, self.t_pherm[to_add[curr - 1]][demand[dem_ind][0]])
    for j in range(dem_ind + 1, len(demand)):
      to_add = self.lep_t[dem_ind][demand[j][0]]
      curr = 1
      while curr < len(to_add):
        self.t_pherm[to_add[curr - 1]][to_add[curr]] -= min(w_i, self.t_pherm[to_add[curr - 1]][to_add[curr]])
        curr += 1
      self.t_pherm[to_add[curr - 1]][demand[dem_ind][0]] -= min(w_i, self.t_pherm[to_add[curr - 1]][demand[dem_ind][0]])
    ind, dst, w_coeff = -1, -1, -1
    # Removing phermones from switch
    # points distribution, removing
    # exploration phermones as well.
    for j in range(len(got)):
      got[j] = 0
    q_ind.append(demand[dem_ind][0])
    q_len.append(0)
    got[demand[dem_ind][0]] = 1
    while len(q_ind) > 0:
      ind = q_ind.pop()
      dst = q_len.pop()
      self.sp_pherm[ind] -= min((1 - abs(1 - (dst / R_HALF))) * SP_PHERM_COEFF, self.sp_pherm[ind])
      got[ind] = 1
      for n_ind, n_dst, _, _ in edges[ind]:
        n_dst += dst
        if n_dst < MAX_DST and got[n_ind] == 0:
          self.t_pherm[ind][n_ind] -= min(T_PATH_EXPLR, self.t_pherm[ind][n_ind])
          self.t_pherm[n_ind][ind] -= min(T_PATH_EXPLR, self.t_pherm[n_ind][ind])
          q_ind.append(n_ind)
          q_len.append(n_dst)
    # Drone & Truck Edges Work:
    heapq.heapify(q_ind)
    for j in range(len(got)):
      got[j] = 0
    let = [float('inf') for _ in range(len(nodes))]
    # Local Dijkstra's Algorithm
    # with phermone loading
    heapq.heappush(q_ind, (0, 0, demand[dem_ind][0]))
    got[demand[dem_ind][0]] = 1
    w_coeff = DT_PATH_BASE + DT_PATH_COEFF * (demand[dem_ind][1] / max_weight)
    while len(q_ind) > 0:
      eng, dst, ind = heapq.heappop(q_ind)
      for e in edges[ind]:
        if got[e[0]] == 1:
          continue
        got[e[0]] = 1
        tent_eng = eng + e[3][0]  # 0.5kg payload
        n_dst = dst + e[1]
        if tent_eng < let[e[0]] and n_dst < MAX_DST:
          self.dt_pherm[e[0]][ind] -= min(w_coeff * (1 - (n_dst / R)), self.dt_pherm[e[0]][ind])
          let[e[0]] = tent_eng
          heapq.heappush(q_ind, (tent_eng, n_dst, e[0]))
    # Drone Only Edges Work:
    heapq.heapify(q_ind)
    for j in range(len(got)):
      got[j] = 0
    for j in range(len(let)):
      let[j] = float('inf')
    # Local Dijkstra's Algorithm
    # with phermone loading
    heapq.heappush(q_ind, (0, 0, demand[dem_ind][0]))
    got[demand[dem_ind][0]] = 1
    w_coeff = DO_PATH_BASE + DO_PATH_COEFF * (demand[dem_ind][1] / max_weight)
    while len(q_ind) > 0:
      eng, dst, ind = heapq.heappop(q_ind)
      for e in dedges[ind]:
        if got[e[0]] == 1:
          continue
        got[e[0]] = 1
        tent_eng = eng + e[2][0]  # 0.5kg payload
        n_dst = dst + e[1]
        if tent_eng < let[e[0]] and n_dst < MAX_DST:
          self.do_pherm[e[0]][ind] -= min(w_coeff * (1 - (n_dst / R)), self.do_pherm[e[0]][ind])
          let[e[0]] = tent_eng
          heapq.heappush(q_ind, (tent_eng, n_dst, e[0]))

  def init_phermone_system(self, R=float("inf")):
    # range is a dummy decision variable for now
    print("Generating phermones tracker...")
    nodes = self.nodes
    edges = self.edges
    dedges = self.dedges
    demand = self.demand
    DEMAND_BASE_PHERM = 0.3
    DEMAND_PHERM_COEFF = 0.2
    NODE_BASE_PHERM = 0
    SP_PHERM_COEFF = 0.7
    got = [0 for _ in range(len(nodes))]
    q_ind = []
    lep, let, eng, ind, tent_eng, w_i = None, None, -1, -1, -1, -1
    up_arrs, curr, next = [], [], -1, -1
    n_pherm = [[NODE_BASE_PHERM for _ in range(len(nodes))] for _ in range(len(demand))]
    max_weight = max(w for _, w in demand)
    for i in range(len(demand)):
      w_i = DEMAND_BASE_PHERM + DEMAND_PHERM_COEFF * (demand[i][1] / max_weight)
      for j in range(i + 1, len(demand)):
        n_pherm[i][demand[j][0]] += w_i + DEMAND_PHERM_COEFF * (demand[j][1] / max_weight)
        n_pherm[j][demand[i][0]] = n_pherm[i][demand[j][0]]
    lep_t = [[-1 for _ in range(len(nodes))] for _ in range(len(demand))]
    let_t = [[float('inf') for _ in range(len(nodes))] for _ in range(len(demand))]
    print("Running Dijkstra's + Path Tracking for all demand nodes...")
    pbar = tqdm(total=len(demand))
    for i in range(len(demand)):
      let_t[i][demand[i][0]] = 0
      for j in range(len(got)):
        got[j] = 0
      lep, let = lep_t[i], let_t[i]
      # Dijkstra's Algorithm
      q_ind.append((0, demand[i][0]))
      got[demand[i][0]] = 1
      while len(q_ind) > 0:
        eng, ind = heapq.heappop(q_ind)
        for e in edges[ind]:
          if got[e[0]] == 1:
            continue
          got[e[0]] = 1
          tent_eng = eng + e[2]
          if tent_eng < let[e[0]]:
            let[e[0]] = tent_eng
            lep[e[0]] = ind
            heapq.heappush(q_ind, (tent_eng, e[0]))
      # Path Construction
      lep[demand[i][0]] = []
      curr = -1
      for j in range(len(nodes)):
        if isinstance(lep[j], int):
          if lep[j] >= 0:
            curr = lep[j]
            lep[j] = [j]
            up_arrs.append(lep[j])
          else:
            lep[j] = []
            continue
        while curr >= 0:
          if isinstance(lep[curr], int):
            for arr in up_arrs:
              arr.append(curr)
            next = lep[curr]
            lep[curr] = [curr]
            if next >= 0:
              up_arrs.append(lep[curr])
            curr = next
          else:
            if len(lep[curr]) > 0:
              for arr in up_arrs:
                arr.extend(lep[curr])
            curr = -1
        up_arrs.clear()
      pbar.update()
    pbar.close()
    print("Dijkstra's + Path Tracking complete!")
    print("Initializing switch point possibilities & local best paths around demands...")
    n_eng = 0
    sp_poss = [[] for _ in range(len(demand))]
    let_g = [float('inf') for _ in range(len(nodes))]
    llep_d = [{} for _ in range(len(demand))]
    pbar = tqdm(total=len(demand))
    for dem_ind in range(len(demand)):
      i, demand_weight = demand[dem_ind]
      if demand_weight > WEIGHTS[-1]:
        continue
      for j in range(len(got)):
        got[j] = 0
      q_ind.append((0, 0, i))
      got[i] = 1
      while len(q_ind) > 0:
        tent_eng, dst, ind = heapq.heappop(q_ind)
        sp_poss[dem_ind].append(ind)
        for n in range(len(demand)):
          n_pherm[n][ind] += (1 - abs(1 - (tent_eng / MAX_BATTERY_USE_HALF))) * SP_PHERM_COEFF
        for n_ind, n_dst, _, _, drone_powers in edges[ind]:
          if got[n_ind] == 1:
            continue
          got[n_ind] = 1
          # loading based on maximum range i.e. 0.25kg payload
          n_eng = tent_eng + (drone_powers[1] * n_dst / DRONE_GROUND_SPEED)
          n_dst += dst
          if n_eng < MAX_BATTERY_USE and got[n_ind] == 0 and n_dst < R:
            heapq.heappush(q_ind, (n_eng, n_dst, n_ind))
      n_pherm[dem_ind][i] = 0
      for j in range(len(got)):
        got[j] = 0
      let = let_g
      for j in range(len(let)):
        let[j] = float('inf')
      lep = llep_d[dem_ind]
      q_ind.append((0, 0, i))
      got[i] = 1
      while len(q_ind) > 0:
        eng, dst, ind = heapq.heappop(q_ind)
        for n_ind, n_dst, _, _, drone_powers in edges[ind]:
          if got[n_ind] == 1:
            continue
          got[n_ind] = 1
          # loading based on maximum range i.e. 0.25kg payload
          n_eng = tent_eng + (drone_powers[1] * n_dst / DRONE_GROUND_SPEED)
          n_dst += dst
          if n_eng < MAX_BATTERY_USE and got[n_ind] == 0 and n_dst < R:
            lep[n_ind] = ind
            heapq.heappush(q_ind, (n_eng, n_dst, n_ind))
        for n_ind, n_dst, drone_powers in dedges[ind]:
          if got[n_ind] == 1:
            continue
          got[n_ind] = 1
          # loading based on maximum range i.e. 0.25kg payload
          n_eng = tent_eng + (drone_powers[1] * n_dst / DRONE_GROUND_SPEED)
          n_dst += dst
          if n_eng < MAX_BATTERY_USE and got[n_ind] == 0 and n_dst < R:
            llep_d[dem_ind][n_ind] = ind
            heapq.heappush(q_ind, (n_eng, n_dst, n_ind))
      lep[i] = []
      curr = -1
      for j in lep:
        if isinstance(lep[j], int):
          curr = lep[j]
          lep[j] = [j]
          up_arrs.append(lep[j])
        while curr >= 0:
          if isinstance(lep[curr], int):
            for arr in up_arrs:
              arr.append(curr)
            next = lep[curr]
            lep[curr] = [curr]
            up_arrs.append(lep[curr])
            curr = next
          else:
            if len(lep[curr]) > 0:
              for arr in up_arrs:
                arr.extend(lep[curr])
            break
        up_arrs.clear()
      pbar.update()
    pbar.close()
    print("Initialization complete!")
    self.sp_poss = sp_poss
    self.n_pherm = n_pherm
    self.llep_d = llep_d
    self.lep_t = lep_t
    self.let_t = let_t
    print("Phermone system initialized!")

  def aco(self, src):
    """
    src: source depot index in self.nodes
    """
    # 1% decrease in mpg for every 100 pounds
    # implies 1 / (1 - 0.01 * num_pounds) multiplier. 
    truck_coeff = 1 / (1 - (0.01 * self.total_weight / 45.359237))

    return 0

def D_f(rho, V):   # correct, but prefer not used
  """
  Drag force.
  TODO: include effect of having a package.
  """
  return 0.5 * C_D_ALPHA0 * S_REF * rho * V * V

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
def power(rho, W, V_w_hd, V_w_lt):
  V = DRONE_GROUND_SPEED
  W += kgs_to_W(13)    # needs update, drone base weight
  Df = 0.5 * C_D_ALPHA0 * S_REF * rho * V * V
  T0, H0 = W / 6, 0.015 * W
  T_OLD, alpha_D, COSALPHD, SINALPHD = 0, 0, 0, 0
  omega0, omegaN = 350, 350
  TBET, HBET, QBET = 0, 0, 0
  Vx, Vc = 0, 0
  # print("Weight:", W)
  resT, resH, resQ = 0, 0, 0
  T, H, Q, x, rdr_SI = 0, 0, 0, 0, 0
  dr_SI = 0.00254
  dr_SI_sq = 0.0000064516
  dx = 0.009090909091
  dv, vCOEFF, vCOEFF_INIT = 0, 0, 0
  Vc_v_sum1, Vc_v_sum2 = 0, 0
  omega_r, d_omega_r = 0, 0
  i, j = 0, 0
  SUM, fx = 0, 0
  phi = 0
  VTsq_c = 0
  l, d = 0, 0
  lCOEFF = rho * 0.00285
  dCOEFF = rho * 0.0000225
  cosphi, sinphi = 0, 0
  HCOEFF = 0
  C, k_v, fv, dv, Vxsq, VAR1 = 0, 0, 0, 0, 0, 0
  while abs(T0 - T_OLD) / T0 > 0.00001:
    T_OLD = T0
    alpha_D = atan2(Df, W) + atan2(H0, sqrt(Df*Df + W*W))
    T0 = sqrt(W*W + Df*Df - H0*H0) / 6
    TBET = T0 / 2
    COSALPHD = cos(alpha_D)
    SINALPHD = sin(alpha_D)
    Vx = (V + V_w_hd) * COSALPHD + V_w_lt * SINALPHD
    Vxsq = Vx * Vx
    Vc = (V + V_w_hd) * SINALPHD + V_w_lt * COSALPHD
    C = T0 / (2 * rho * AREA)
    C *= C
    k_v = 2
    VAR1 = k_v + Vc
    fv = ((k_v * k_v * (VAR1 * VAR1 + Vxsq)) - C)
    dv = fv / (2 * k_v * ((VAR1 * (VAR1 + k_v)) + Vxsq))
    while abs(fv) > 0.0001:
      k_v -= dv
      VAR1 = k_v + Vc
      fv = ((k_v * k_v * (VAR1 * VAR1 + Vxsq)) - C)
      dv = fv / (2 * k_v * ((VAR1 * (VAR1 + k_v)) + Vxsq))
    # print("alpha_d:",rad_to_deg(alpha_D),"T0:",T0,"v0:",k_v,"Vx:",Vx,"Vc:",Vc)
    while abs(TBET - T0) / T0 > 0.0001:
      resT, resH, resQ = 0, 0, 0
      i = 0
      vCOEFF_INIT = tan(atan2(Vx, k_v + Vc) / 2)
      d_omega_r = omega0 * dr_SI
      while i < 100:
        T, H, Q, j = 0, 0, 0, 0
        x = 0.09090909091
        rdr_SI = 0.000064516
        omega_r = omega0 * 0.0254
        vCOEFF = vCOEFF_INIT * COSPSI[i]
        dv = k_v * dx * vCOEFF
        SUM = x * vCOEFF
        Vc_v_sum1 = Vc + k_v * (1 + SUM)
        Vc_v_sum2 = Vc + k_v * (1 - SUM)
        Vx_sinpsi = Vx * SINPSI[i]
        HCOEFF = SINPSI[i] * dr_SI
        while j < 100:
          SUM = omega_r + Vx_sinpsi
          if SUM > 0:
            phi = atan2(Vc_v_sum1, SUM)
            VTsq_c = (Vc_v_sum1 * Vc_v_sum1 + SUM * SUM) * CHORD[j]
            l = lCOEFF * min(0.2, max(BETA[j] - phi, -0.2)) * VTsq_c
            d = dCOEFF * VTsq_c
            cosphi, sinphi = cos(phi), sin(phi)
            T += (l*cosphi - d*sinphi) * dr_SI
            fx = (l*sinphi + d*cosphi)
            H += fx * HCOEFF
            Q += fx * rdr_SI
          SUM = omega_r - Vx_sinpsi
          if SUM > 0:
            phi = atan2(Vc_v_sum2, SUM)
            VTsq_c = (Vc_v_sum2 * Vc_v_sum2 + SUM * SUM) * CHORD[j]
            l = lCOEFF * min(0.2, max(BETA[j] - phi, -0.2)) * VTsq_c
            d = dCOEFF * VTsq_c
            cosphi, sinphi = cos(phi), sin(phi)
            T += (l*cosphi - d*sinphi) * dr_SI
            fx = (l*sinphi + d*cosphi)
            H -= fx * HCOEFF
            Q += fx * rdr_SI
          x += dx
          rdr_SI += dr_SI_sq
          Vc_v_sum1 += dv
          Vc_v_sum2 -= dv
          omega_r += d_omega_r
          j += 1
        resT += T
        resH += H
        resQ += Q
        i += 1
      TBET, HBET, QBET = resT * 0.01, resH * 0.01, resQ * 0.01
      omegaN = sqrt(T0/TBET) * omega0
      # print("QBET:",QBET,"TBET:",TBET,"HBET:",HBET,"O0:",omega0,"ON:",omegaN)
      omega0 = omegaN
    T0 = TBET
    H0 = HBET
  # print((W/6)-T0) # excess
  # omega_to_RPM(omegaN), rad_to_deg(alpha_D)
  return (omegaN * QBET * 7.05882353)
  # assumes each of the 6 motors has 85% efficiency.

def fill_edge_data(edgesl, dedges, edge_work, dedge_work):
  print("Logical number of CPUs:", cpu_count())
  p = mp.Pool(processes=cpu_count(), 
              initializer=copy_globals,
              initargs=(DRONE_GROUND_SPEED,))
  pbar = tqdm(total=(len(edge_work) + len(dedge_work)))
  def update(*a):
    pbar.update()
  u_ind, v_ind, length = 0, 0, 0
  rho, V_w_hd, V_w_lt = 0, 0, 0
  async_obj, ind = 0, 0
  truck_energy, truck_speed = 0, 0
  for i in range(len(edge_work)):
    u_ind, ind, rho, V_w_hd, V_w_lt = edge_work[i]
    edge_work[i] = (u_ind, ind, p.apply_async(_worker, (rho, V_w_hd, V_w_lt), callback=update))
  for i in range(len(dedge_work)):
    u_ind, ind, rho, V_w_hd, V_w_lt = dedge_work[i]
    dedge_work[i] = (u_ind, ind, p.apply_async(_worker, (rho, V_w_hd, V_w_lt), callback=update))
  for i in range(len(edge_work)):
    u_ind, ind, async_obj = edge_work[i]
    v_ind, length, truck_energy, truck_speed = edgesl[u_ind][ind]
    edgesl[u_ind][ind] = (v_ind, length, truck_energy, truck_speed, async_obj.get())
  for i in range(len(dedge_work)):
    u_ind, ind, async_obj = dedge_work[i]
    v_ind, length = dedges[u_ind][ind]
    dedges[u_ind][ind] = (v_ind, length, async_obj.get())
  p.close()
  p.join()
  pbar.close()

def _worker(rho, V_w_hd, V_w_lt):
  drone_power = []
  for w in WEIGHTS:
    drone_power.append(power(rho, w, V_w_hd, V_w_lt))
  return tuple(drone_power)
