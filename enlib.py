from math import sqrt, acos, atan2, pi, \
                 sin, cos, tan, exp, \
                 floor, ceil
from random import randint, sample, choices
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
BATTERY_CAPACITY = 17.0 * 42 * 360  # J,                           TODO: CHANGE 360 -> 3600
MAX_BATTERY_USE = BATTERY_CAPACITY * (1 - BATTERY_RESERVE_MARGIN)
MAX_BATTERY_USE_HALF = MAX_BATTERY_USE / 2
MIN_MEETUP_BATTERY_REM = MAX_BATTERY_USE * 0.15
TIME_TO_FULL_CHARGE = 3600
RECARGE_RATE = BATTERY_CAPACITY / TIME_TO_FULL_CHARGE
C_D_ALPHA0, S_REF, DRONE_GROUND_SPEED = -1, -1, -1
CHORD, BETA, SINPSI, COSPSI = -1, -1, -1, -1
MAX_TRUCK_SPEED, BASE_TRUCK_SPEED, TRUCK_CITY_MPG = -1, -1, -1
BASE_TEMP, TEMP_FLUC_COEFF, REL_HUMIDITY = -1, -1, -1
QUAD_A, QUAD_B, QUAD_C = -1, -1, -1
WEIGHTS = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2]
DRONE_WEIGHT = -1
ENG_ZERO = 10**(-50)

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
         TEMP_FLUC_COEFF, REL_HUMIDITY, QUAD_A, QUAD_B, QUAD_C, DRONE_WEIGHT
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
  DRONE_WEIGHT = kgs_to_W(12)

def copy_globals_energy(drone_speed):
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
    tgt_edge, e = None, None
    q, to_add = [], []
    got = [0 for _ in range(len(self.nodes))]
    print("Finding edges which need to be added to ensure well-connectedness of drive network...")
    count = 0
    pbar = tqdm(total=len(self.nodes))
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
        self.edges[tgt_edge[0]].append((n, tgt_edge[1], tgt_edge[2], tgt_edge[3], tgt_edge[4]))
      pbar.update()
    pbar.close()
    print(round(count / sum(len(e) for e in self.edges), 2), "percent road connections were made both-way!")

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

  def plot_cycle(self, cycle, swp, save=False, pic_number=0):
    got = [0 for _ in range(len(self.nodes))]
    nx, ny, nc = [], [], []
    dx, dy = [], []
    max_pherm = 0
    for p in self.demand:
      got[p[0]] = 1
      dx.append(self.nodes[p[0]][0])
      dy.append(self.nodes[p[0]][1])
    if self.n_pherm is not None and len(self.demand) > 0:
      max_pherm = max(max(p for p in self.n_pherm[i]) for i in range(len(self.demand)))
      base_c = 0.01 * max_pherm
      comp = 0
      for i in range(len(self.nodes)):
        if got[i] == 1:
          continue
        nx.append(self.nodes[i][0])
        ny.append(self.nodes[i][1])
        if len(self.edges[i]) > 0:
          comp = 0.9 - min(1.75 * ((1000 * self.n_pherm[0][i] + base_c) / max_pherm), 0.9)
          nc.append((1, comp, comp))
        else:
          nc.append((0.7, 0.7, 0.7))
    else:
      for i in range(len(self.nodes)):
        if got[i] == 1:
          continue
        nx.append(self.nodes[i][0])
        ny.append(self.nodes[i][1])
        if len(self.edges[i]) > 0:
          nc.append((1, 0, 0))
        else:
          nc.append((0.7, 0.7, 0.7))
    if self.line_cover is None:
      self.line_cover = self.gen_network_line_cover(self.edges)
    if self.line_cover_d is None:
      self.line_cover_d = self.gen_network_line_cover(self.dedges)
    plt.scatter(dx, dy, c="magenta", s=15)
    plt.scatter(x=nx, y=ny, color=nc, s=8)
    llx, lly, lln = self.line_cover
    for i in range(len(llx)):
      plt.plot(llx[i], lly[i], marker="", c="mediumblue", alpha=0.3)
    llx, lly, lln = self.line_cover_d
    for i in range(len(llx)):
      plt.plot(llx[i], lly[i], marker="", c="limegreen", alpha=0.3)
    glx, gly = [], []
    lx, ly = [], []
    for ind in self.lep_t[-1][self.demand[cycle[0]][0]]:
      lx.append(self.nodes[ind][0])
      ly.append(self.nodes[ind][1])
    lx.append(self.nodes[self.demand[-1][0]][0])
    ly.append(self.nodes[self.demand[-1][0]][1])
    lx.reverse()
    ly.reverse()
    glx.extend(lx)
    gly.extend(ly)
    dem_ind = 0
    while dem_ind < len(cycle) - 1:
      lx, ly = [], []
      path = self.lep_t[cycle[dem_ind]][self.demand[cycle[dem_ind + 1]][0]]
      for i in range(len(path)):
        lx.append(self.nodes[path[i]][0])
        ly.append(self.nodes[path[i]][1])
      lx.append(self.nodes[self.demand[cycle[dem_ind]][0]][0])
      ly.append(self.nodes[self.demand[cycle[dem_ind]][0]][1])
      lx.reverse()
      ly.reverse()
      glx.extend(lx)
      gly.extend(ly)
      dem_ind += 1
    lx, ly = [], []
    for ind in self.lep_t[cycle[-1]][self.demand[-1][0]]:
      lx.append(self.nodes[ind][0])
      ly.append(self.nodes[ind][1])
    lx.append(self.nodes[self.demand[cycle[-1]][0]][0])
    ly.append(self.nodes[self.demand[cycle[-1]][0]][1])
    lx.reverse()
    ly.reverse()
    glx.extend(lx)
    gly.extend(ly)
    plt.plot(glx, gly, marker="", c="black", alpha=0.6)
    if save:
      plt.savefig("Pictures/{}.png".format(pic_number), dpi=500)
      plt.clf()

  def plot_network(self, show_drone_only_nodes, show_drone_only_edges, show_demand_nodes, 
                   show_demand_local_paths, show_for_all_edges, spec_ind=[], spec_path=[]):
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
    if self.n_pherm is not None and len(self.demand) > 0:
      max_pherm = max(max(p for p in self.n_pherm[i]) for i in range(len(self.demand)))
      base_c = 0.01 * max_pherm
      comp = 0
      if show_drone_only_nodes:
        for i in range(len(self.nodes)):
          if got[i] == 1:
            continue
          nx.append(self.nodes[i][0])
          ny.append(self.nodes[i][1])
          if len(self.edges[i]) > 0:
            comp = 0.9 - min(1.75 * ((1000 * self.n_pherm[0][i] + base_c) / max_pherm), 0.9)
            nc.append((1, comp, comp))
          else:
            nc.append((0.7, 0.7, 0.7))
      else:
        for i in range(len(self.nodes)):
          if got[i] == 1 or len(self.edges[i]) == 0:
            continue
          nx.append(self.nodes[i][0])
          ny.append(self.nodes[i][1])
          comp = 0.9 - min(1.75 * ((1000 * self.n_pherm[0][i] + base_c) / max_pherm), 0.9)
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
    if show_demand_local_paths:
      for i in range(len(self.demand)):
        dem_ind = self.demand[i][0]
        for k in self.llep_d[i]:
          lx, ly = [], []
          for ind in self.llep_d[i][k]:
            lx.append(self.nodes[ind][0])
            ly.append(self.nodes[ind][1])
          lx.append(self.nodes[dem_ind][0])
          ly.append(self.nodes[dem_ind][1])
          plt.plot(lx, ly, marker="", c="black", alpha=0.3)
    if show_for_all_edges:
      llx, lly, lln = self.line_cover
      for i in range(len(llx)):
        plt.plot(llx[i], lly[i], marker="", c="mediumblue", alpha=0.4)
    if show_drone_only_edges:
      llx, lly, lln = self.line_cover_d
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

  def verify_demand_connectivity(self, st_ind, q, got):
    for i in range(len(got)):
      got[i] = 0
    q.append(st_ind)
    got[st_ind] = 1
    ind = -1
    while len(q) > 0:
      ind = q.pop()
      for e in self.edges[ind]:
        if got[e[0]] == 0:
          got[e[0]] = 1
          q.append(e[0])
    if len(self.demand) > 0:
      if isinstance(self.demand[0], tuple):
        for i in range(len(self.demand)):
          if got[self.demand[i][0]] == 0:
            return False
      else:
        for i in range(len(self.demand)):
          if got[self.demand[i]] == 0:
            return False
    return True

  def append_random_demand(self, num, cluster_num=0, cluster_jump=0, 
                           drone_only_possible_component=0.7, num_allocs=10):
    """
    Demand generated ALWAYS corresponds
    to nodes reachable by the truck.

    drone_only_possible_component is
    responsible for allocating pure
    drone only possible deliveries
    """
    if num == 0:
      return
    print("Generating demand...")
    N, num_clone = len(self.nodes), num
    ind, tgt, cnt = -1, -1, -1
    q, data = [], None
    got = [0 for _ in range(N)]
    cnt, lim, lim2 = 0, N, -1
    i = randint(0, N - 1)
    while lim > 0:
      collec = []
      for j in range(len(got)):
        got[j] = 0
      q.append(i)
      got[i] = 1
      while len(q) > 0:
        ind = q.pop(0)
        collec.append(ind)
        for e in self.edges[ind]:
          if got[e[0]] == 0:
            got[e[0]] = 1
            q.append(e[0])
      if len(collec) > cnt:
        cnt = len(collec)
      if cnt >= num:
        data = collec
        break
      lim -= 1
      i = randint(0, N - 1)
    print("Quick Directional BFS discovered", round(100 * len(data) / N, 2), "percent of stored nodes.")
    if len(data) < num:
      print("ERROR: couldn't find enough reachable nodes to generate demand.")
      exit(0)
    _got = [0 for _ in range(N)]
    for i in range(len(got)):
      got[i] = 0
    if cluster_num > 0:
      ind = 0
      lim2 = N
      tgt = ceil(num / cluster_num)
      while tgt > 0 and lim2 > 0:
        num -= tgt
        while tgt > 0:
          lim = N
          while got[data[ind]] == 1 and lim > 0:
            ind = (ind + 1) % len(data)
            lim -= 1
          got[data[ind]] = 1
          if self.verify_demand_connectivity(data[ind], q, _got):
            self.demand.append(data[ind])
            ind = (ind + cluster_jump) % len(data)
            tgt -= 1
          else:
            lim2 -= 1
            break
        num += tgt
        tgt = ceil(num / cluster_num)
        ind = randint(0, len(data) - 1)
    else:
      ind = 0
      lim2 = N
      while num > 0 and lim2 > 0:
        lim = N
        while got[data[ind]] == 1 and lim > 0:
          ind = (ind + 1) % len(data)
          lim -= 1
        got[data[ind]] = 1
        if self.verify_demand_connectivity(data[ind], q, _got):
          self.demand.append(data[ind])
          num -= 1
        else:
          lim2 -= 1
        ind = randint(0, len(data) - 1)
    if num > 0:
      print("Error: Failed to generate well connected data from available nodes.")
    w_total, w = 0, 0
    w_a, w_b = 1, len(WEIGHTS) - 1 
    iters = [i for i in range(len(self.demand) - num_clone, len(self.demand))]
    pure_drone = sample(iters, ceil(drone_only_possible_component * num_clone))
    for i in iters:
      w = 0
      if i in pure_drone:
        w = WEIGHTS[randint(w_a, w_b)]
      else:
        w = 0
        for _ in range(num_allocs):
          w += WEIGHTS[randint(w_a, w_b)]
      self.demand[i] = (self.demand[i], w)
      w_total += w
    self.total_weight += w_total
    if self.total_weight >= 4500:
      print("WARNING: Total demand weight exceeds 4500kg critical point.")
    print("Demand generated!")

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
      if self.nodes[i][0] > max_x and \
         self.nodes[i][1] > max_y and \
         len(self.edges[i]) > 0:
        max_x = self.nodes[i][0]
        max_y = self.nodes[i][1]
        ind = i
    return ind

  def get_top_left_node(self):
    max_x, min_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] > max_x and \
         self.nodes[i][1] < min_y and \
         len(self.edges[i]) > 0:
        max_x = self.nodes[i][0]
        min_y = self.nodes[i][1]
        ind = i
    return ind

  def get_bottom_right_node(self):
    min_x, max_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] < min_x and \
         self.nodes[i][1] > max_y and \
         len(self.edges[i]) > 0:
        min_x = self.nodes[i][0]
        max_y = self.nodes[i][1]
        ind = i
    return ind

  def get_bottom_left_node(self):
    min_x, min_y, ind = 0, 0, 0
    for i in range(len(self.nodes)):
      if self.nodes[i][0] < min_x and \
         self.nodes[i][1] < min_y and \
         len(self.edges[i]) > 0:
        min_x = self.nodes[i][0]
        min_y = self.nodes[i][1]
        ind = i
    return ind

  def init_phermone_system(self, src, passed_num_alloc, R=float("inf")):
    # range is a dummy decision variable for now
    print("Initializng phermone system...")
    nodes, edges, dedges, demand = self.nodes, self.edges, self.dedges, self.demand
    demand.append((src, 0))
    NODE_BASE_PHERM = 10**(-6)
    SP_PHERM_COEFF = 10**(-5)
    DEMAND_PHERM_ADV_COEFF = 100
    DEMAND_BASE_PHERM = DEMAND_PHERM_ADV_COEFF * SP_PHERM_COEFF * len(demand)
    DEMAND_WEIGHT_COEFF = DEMAND_PHERM_ADV_COEFF * NODE_BASE_PHERM / (WEIGHTS[-1] * passed_num_alloc)
    n_pherm = [[NODE_BASE_PHERM for _ in range(len(nodes))] for _ in range(len(demand))]
    got = [0 for _ in range(len(nodes))]
    let_g = [float('inf') for _ in range(len(nodes))]
    lep_g = [(-1, None) for _ in range(len(nodes))]
    # [0]: from demand to node, [1]: from node to demand
    q, lep, let, lep2, eng, ind, cur, prv, ed_ind = [], lep_g, let_g, None, -1, -1, -1, -1, -1
    sp_poss = [([],[]) for _ in range(len(demand))]
    llep_d = [({},{}) for _ in range(len(demand))]
    # initializing paths towards all demand
    print("Initializing switch point possibilities & local best paths around demands...")
    pbar = tqdm(total=len(nodes))
    for i in range(len(nodes)):
      let[i] = ENG_ZERO
      q.append((ENG_ZERO, 0, i))
      while len(q) > 0:
        eng, dst, ind = heapq.heappop(q)
        if got[ind] == 1:
          continue
        got[ind] = 1
        for n_ind, n_dst, _, _, drone_powers in edges[ind]:
          # loading based on maximum range i.e. 0.25kg payload
          n_eng = eng + (drone_powers[1] * n_dst / DRONE_GROUND_SPEED)
          n_dst += dst
          if n_eng < MAX_BATTERY_USE and n_eng < let[n_ind] and n_dst < R:
            let[n_ind] = n_eng
            lep[n_ind] = (ind, True)
            heapq.heappush(q, (n_eng, n_dst, n_ind))
        for n_ind, n_dst, drone_powers in dedges[ind]:
          # loading based on maximum range i.e. 0.25kg payload
          n_eng = eng + (drone_powers[1] * n_dst / DRONE_GROUND_SPEED)
          n_dst += dst
          if n_eng < MAX_BATTERY_USE and n_eng < let[n_ind] and n_dst < R:
            let[n_ind] = n_eng
            lep[n_ind] = (ind, False)
            heapq.heappush(q, (n_eng, n_dst, n_ind))
      for j in range(len(demand)):
        if got[demand[j][0]] == 1:
          sp_poss[j][1].append(i)
          lep2 = llep_d[j][1]
          prv = demand[j][0]
          cur, cur_ty = lep[demand[j][0]]
          eng = let[demand[j][0]]
          while cur != -1:
            if cur in lep2:
              break
            n_pherm[j][cur] += (1 - abs(1 - ((eng - let[cur]) / MAX_BATTERY_USE_HALF))) * SP_PHERM_COEFF
            ed_ind = 0
            edge_data = edges
            if not cur_ty:
              edge_data = dedges
            while edge_data[cur][ed_ind][0] != prv:
              ed_ind += 1
            lep2[cur] = (ed_ind, cur_ty)
            prv = cur
            cur, cur_ty = lep[cur]
        if i == demand[j][0]:
          sp_poss[j][0].append(i)
          lep2 = llep_d[j][0]
          for k1 in range(len(nodes)):
            if lep[k1][0] != -1:
              sp_poss[j][0].append(k1)
              cur, cur_ty = lep[k1]
              edge_data = edges
              if not cur_ty:
                edge_data = dedges
              for k2 in range(len(edge_data[cur])):
                if edge_data[cur][k2][0] == k1:
                  lep2[k1] = (cur, k2, cur_ty)
                  break
      for j in range(len(nodes)):
        got[j] = 0
        let[j] = float("inf")
        lep[j] = (-1, None)
      pbar.update()
    for j in range(len(demand)):
      llep_d[j][0][demand[j][0]] = (-1, -1, None)
      llep_d[j][1][demand[j][0]] = (-1, None)
    pbar.close()
    print("Initialization for demand complete!")
    print("Finding shortest paths between all drive intersections...")
    lep_t = [[(-1, -1) for _ in range(len(nodes))] for _ in range(len(nodes))]
    let_t = [[float('inf') for _ in range(len(nodes))] for _ in range(len(nodes))]
    pbar = tqdm(total=len(nodes))
    for i in range(len(nodes)):
      for j in range(len(got)):
        got[j] = 0
      lep, let = lep_t[i], let_t[i]
      let[i] = ENG_ZERO
      # Dijkstra's Algorithm
      q.append((0, i))
      while len(q) > 0:
        eng, ind = heapq.heappop(q)
        if got[ind] == 1:
          continue
        got[ind] = 1
        for j in range(len(edges[ind])):
          e = edges[ind][j]
          n_eng = eng + e[2]
          if n_eng < let[e[0]]:
            let[e[0]] = n_eng
            lep[e[0]] = (ind, j)
            heapq.heappush(q, (n_eng, e[0]))
      pbar.update()
    pbar.close()
    print("Complete!")
    print("Verifying connections and setting phermones...")
    for i in range(len(demand)):
      for j in range(i + 1, len(demand)):
        if let_t[self.demand[i][0]][self.demand[j][0]] == float("inf"):
          print("BAD CONNECTION FOUND:", self.demand[i][0], self.demand[j][0], ". Aborting...")
          exit(1)
        if let_t[self.demand[j][0]][self.demand[i][0]] == float("inf"):
          print("BAD CONNECTION FOUND:", self.demand[j][0], ",", self.demand[i][0], ". Aborting...")
          exit(1)
        n_pherm[i][demand[j][0]] += DEMAND_BASE_PHERM + DEMAND_WEIGHT_COEFF * demand[j][1]
        n_pherm[j][demand[i][0]] = n_pherm[i][demand[j][0]]
    print("Verified connections and set phermones!")
    self.sp_poss = sp_poss
    self.n_pherm = n_pherm
    self.llep_d = llep_d
    self.lep_t = lep_t
    self.let_t = let_t
    print("Phermone system initialized!\nNOTE: Demand structures now hold source vertex with 0 weight.")

  def aco_truck_only(self, K=150, ants_per_iter=50, q=10**6, degradation_factor=0.99):
    print("Initializing ACO child workers...")
    if cpu_count() < ants_per_iter:
      ants_per_iter = cpu_count()
      print("WARNING: cpu count too low, set ants/iteration to cpu count:", cpu_count())
    STAGNANT_LIMIT = int(0.3 * K)
    BEST_HALF_SIZE = ants_per_iter // 2
    degradation_factor = degradation_factor**BEST_HALF_SIZE
    barrier = mp.Value('i',lock=True)
    with barrier.get_lock():
      barrier.value = ants_per_iter
    saw_zero = mp.Value('i',lock=True)
    with saw_zero.get_lock():
      saw_zero.value = 0
    demand = self.demand    # not changing
    DEMAND_SIZE = len(demand) - 1
    NUM_NODES = len(self.nodes)
    N_PHERM_LAST = int(DEMAND_SIZE * NUM_NODES)
    N_PHERM_SIZE = N_PHERM_LAST + NUM_NODES
    n_pherm = mp.Array('f', N_PHERM_SIZE, lock=False)
    c = -1
    for i in range(DEMAND_SIZE + 1):
      c = i * len(self.nodes)
      for j in range(len(self.nodes)):
        n_pherm[c + j] = self.n_pherm[i][j]
    cycles = [(mp.Array('i', DEMAND_SIZE, lock=False),
               mp.Value('f', lock=False)) for _ in range(ants_per_iter)]
    let_t = self.let_t      # not changing
    processes = [mp.Process(target=_aco_worker_truck_only,
                            args=(barrier, saw_zero, demand, n_pherm, 
                                  cycles[i][0], let_t, K, cycles[i][1])) for i in range(ants_per_iter)]
    print("Initialized ACO child workers!\nStarting ACO...")
    best_cycle = sample([i for i in range(DEMAND_SIZE)], k=DEMAND_SIZE)
    best_energy = float(10**90)
    best_cycle_ind, j, delta = -1, -1, -1
    cycle = None
    for p in processes:
      p.start()
    pbar = tqdm(total=K)
    for iter in range(K):
      c = 1
      while c > 0:
        with barrier.get_lock():
          c = barrier.value
      with saw_zero.get_lock():
        saw_zero.value += ants_per_iter
      # if iter % 10 == 0:
      #   self.plot_cycle(cycles[0][0], int(iter / 10))   # for saving pictures
      cycles.sort(key = lambda x: x[1].value)
      if abs(cycles[0][1].value - best_energy) / best_energy < NEWT_PREC:
        STAGNANT_LIMIT -= 1
        if STAGNANT_LIMIT <= 0:
          n_pherm[0] = -1
          c = 1
          while c > 0:
            with saw_zero.get_lock():
              c = saw_zero.value
          with barrier.get_lock():
            barrier.value = ants_per_iter
          print("Limit for iterations to stay stagnant exceeded! Stopping earlier by", K - iter,"iterations")
          break
      delta = q / best_energy
      n_pherm[N_PHERM_LAST + demand[best_cycle[0]][0]] += delta
      j = 0
      while j < DEMAND_SIZE - 1:
        n_pherm[demand[best_cycle[j + 1]][0] + NUM_NODES * best_cycle[j]] += delta
        j += 1
      for i in range(BEST_HALF_SIZE):
        cycle = cycles[i][0]
        if cycles[i][1].value < best_energy:
          best_cycle_ind = i
          best_energy = cycles[i][1].value
        delta = q / cycles[i][1].value
        n_pherm[N_PHERM_LAST + demand[cycle[0]][0]] += delta
        j = 0
        while j < DEMAND_SIZE - 1:
          n_pherm[demand[cycle[j + 1]][0] + NUM_NODES * cycle[j]] += delta
          j += 1
      cycle = cycles[best_cycle_ind][0]
      for i in range(DEMAND_SIZE):
        best_cycle[i] = cycle[i]
      j = 0
      while j < N_PHERM_SIZE:
        n_pherm[j] *= degradation_factor
        j += 1
      pbar.update()
      c = 1
      while c > 0:
        with saw_zero.get_lock():
          c = saw_zero.value
      with barrier.get_lock():
        barrier.value = ants_per_iter
    pbar.close()
    print("ACO complete!")
    for p in processes:
      p.join()
      p.close()
    return best_energy, best_cycle

  def aco(self, K=100, ants_per_iter=50, q=10**6, degradation_factor=0.99):
    """
    Switch Point Communication System:

    first index:  sp | -1
    last index:   sp | -1
    Indicates drone launched / taken back at switch point 'sp' 
    respectively, or -1 if no drone launch / take back.

    (i, i+1) pair of indices such that both not first or
    last index:

    1) (sp, -1):  
        Drone taken back at 'sp' and truck continued to demand.

    2) (-1, sp):
        Drone launched at 'sp', following may be a sequence of
        (-2, sp). End marked by some other type of pair.

    3) (sp1, sp2):
        Drone taken back at sp1, launched again at sp2.

    4) (-2, -2):
        Truck doing deliveries on the side as drone completes
        deliveries simultaneously.

    5) (-2, -1):
        Truck doing deliveries with drone inside it.

    6) (-2, sp):
        Drone doing deliveries from current to next demand,
        via 'sp'
    """
    print("Initializing ACO child workers...")
    if cpu_count() < ants_per_iter:
      ants_per_iter = cpu_count()
      print("WARNING: cpu count too low, set ants/iteration to cpu count:", cpu_count())
    STAGNANT_LIMIT = int(0.2 * K)
    BEST_HALF_SIZE = ants_per_iter // 2
    degradation_factor = degradation_factor**BEST_HALF_SIZE
    barrier = mp.Value('i',lock=True)
    with barrier.get_lock():
      barrier.value = ants_per_iter
    saw_zero = mp.Value('i',lock=True)
    with saw_zero.get_lock():
      saw_zero.value = 0
    demand = self.demand    # not changing
    DEMAND_SIZE = len(demand) - 1
    NUM_NODES = len(self.nodes)
    N_PHERM_LAST = int(DEMAND_SIZE * NUM_NODES)
    N_PHERM_SIZE = N_PHERM_LAST + NUM_NODES
    SWP_SIZE = 2 * DEMAND_SIZE
    SP_PHERM_SIZE = DEMAND_SIZE * DEMAND_SIZE
    SP_PHERM_INIT, DELTA_SP_COEFF = 0.1, 0.75
    n_pherm = mp.Array('f', N_PHERM_SIZE, lock=False)
    sp_pherm = mp.Array('f', SP_PHERM_SIZE, lock=False)
    c = -1
    for i in range(DEMAND_SIZE + 1):
      c = i * NUM_NODES
      for j in range(NUM_NODES):
        n_pherm[c + j] = self.n_pherm[i][j]
    for i in range(SP_PHERM_SIZE):
      sp_pherm[i] = SP_PHERM_INIT
    cycles = [(mp.Value('f', lock=False),
               mp.Array('i', DEMAND_SIZE, lock=False),
               mp.Array('i', SWP_SIZE, lock=False)) for _ in range(ants_per_iter)]
    let_t = self.let_t      # not changing
    sp_poss = self.sp_poss  # not changing
    llep_d = self.llep_d    # not changing
    lep_t = self.lep_t      # not changing
    edges = self.edges      # not changing
    dedges = self.dedges    # not changing
    processes = [mp.Process(target=_aco_worker,
                            args=(barrier, saw_zero, demand, sp_poss, n_pherm, sp_pherm,
                                  cycles[i][1], llep_d, lep_t, cycles[i][2], let_t, K, 
                                  DRONE_GROUND_SPEED, edges, dedges, cycles[i][0])) for i in range(ants_per_iter)]
    print("Initialized ACO child workers!\nStarting ACO...")
    best_cycle = sample([i for i in range(DEMAND_SIZE)], k=DEMAND_SIZE)
    best_swp = [-3 for _ in range(SWP_SIZE)]  # ensures no initial activation
    best_energy = float(10**95)
    best_ind, j, delta, delta_sp, delta_sp_hlf, cyc_ind = -1, -1, -1, -1, -1, -1
    shft, prev_tsj = -1, False
    cycle, swp, energy, last_drone_del = None, None, None, None
    for p in processes:
      p.start()
    pbar = tqdm(total=K)
    for iter in range(K):
      c = 1
      while c > 0:
        with barrier.get_lock():
          c = barrier.value
      with saw_zero.get_lock():
        saw_zero.value += ants_per_iter
      # if iter % 10 == 0:
      #   self.plot_cycle(cycles[0][0], int(iter / 10))   # for saving change pictures
      cycles.sort(key = lambda x: x[0].value)
      if abs(cycles[0][0].value - best_energy) / best_energy < NEWT_PREC:
        STAGNANT_LIMIT -= 1
        if STAGNANT_LIMIT <= 0:
          n_pherm[0] = -1
          c = 1
          while c > 0:
            with saw_zero.get_lock():
              c = saw_zero.value
          with barrier.get_lock():
            barrier.value = ants_per_iter
          print("Limit for iterations to stay stagnant exceeded! Stopping earlier by", K - iter,"iterations")
          break
      # elitism loading below
      delta = q / best_energy
      delta_sp = DELTA_SP_COEFF * delta
      delta_sp_hlf = delta_sp * 0.5
      first_parent, last_drone_del = None, None
      cycle, swp = best_cycle, best_swp
      n_pherm[N_PHERM_LAST + demand[cycle[0]][0]] += delta
      if swp[0] >= 0:
        n_pherm[N_PHERM_LAST + swp[0]] += delta_sp
        for e in edges[swp[0]]:
          if len(edges[e[0]]) > 0:
            n_pherm[N_PHERM_LAST + e[0]] += delta_sp_hlf
        first_parent = cycle[0]
        last_drone_del = first_parent
      j = 1
      cyc_ind = 0
      prev_tsj = False
      while j < SWP_SIZE - 1:
        if swp[j] == -2:
          if swp[j+1] == -2:
            if prev_tsj:
              n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
            else:
              sp_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
              prev_tsj = True
          elif swp[j+1] == -1:
            n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
          else:
            shft = NUM_NODES * cycle[cyc_ind]
            last_drone_del = cycle[cyc_ind + 1]
            n_pherm[demand[last_drone_del][0] + shft] += delta
            n_pherm[swp[j+1] + shft] += delta_sp
            for e in edges[swp[j+1]]:
              if len(edges[e[0]]) > 0:
                n_pherm[e[0] + shft] += delta_sp_hlf
        elif swp[j] >= 0:
          n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * last_drone_del] += delta
          if prev_tsj:
            n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
            n_pherm[swp[j] + NUM_NODES * cycle[cyc_ind]] += delta_sp
            for e in edges[swp[j]]:
              if len(edges[e[0]]) > 0:
                n_pherm[e[0] + shft] += delta_sp_hlf
          else:
            sp_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
          if swp[j+1] >= 0:
            n_pherm[swp[j+1] + NUM_NODES * last_drone_del] += delta_sp
            for e in edges[swp[j+1]]:
              if len(edges[e[0]]) > 0:
                n_pherm[e[0] + shft] += delta_sp_hlf
            last_drone_del = cycle[cyc_ind + 1]
          else:
            last_drone_del = None
          prev_tsj = False
        else:
          first_parent = cycle[cyc_ind + 1]
          last_drone_del = first_parent
          shft = NUM_NODES * cycle[cyc_ind]
          n_pherm[demand[cycle[cyc_ind + 1]][0] + shft] += delta
          n_pherm[swp[j+1] + shft] += delta_sp
          for e in edges[swp[j+1]]:
              if len(edges[e[0]]) > 0:
                n_pherm[e[0] + shft] += delta_sp_hlf
        j += 2
        cyc_ind += 1
      # loading from top 50% of population
      for i in range(BEST_HALF_SIZE):
        energy, cycle, swp = cycles[i]
        energy = energy.value
        if energy < best_energy:
          best_ind = i
          best_energy = energy
        delta = q / energy
        delta_sp = DELTA_SP_COEFF * delta
        delta_sp_hlf = delta_sp * 0.5
        first_parent, last_drone_del = None, None
        n_pherm[N_PHERM_LAST + demand[cycle[0]][0]] += delta
        if swp[0] >= 0:
          n_pherm[N_PHERM_LAST + swp[0]] += delta_sp
          for e in edges[swp[0]]:
            if len(edges[e[0]]) > 0:
              n_pherm[N_PHERM_LAST + e[0]] += delta_sp_hlf
          first_parent = cycle[0]
          last_drone_del = first_parent
        j = 1
        cyc_ind = 0
        prev_tsj = False
        while j < SWP_SIZE - 1:
          if swp[j] == -2:
            if swp[j+1] == -2:
              if prev_tsj:
                n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
              else:
                sp_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
                prev_tsj = True
            elif swp[j+1] == -1:
              n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
            else:
              shft = NUM_NODES * cycle[cyc_ind]
              last_drone_del = cycle[cyc_ind + 1]
              n_pherm[demand[last_drone_del][0] + shft] += delta
              n_pherm[swp[j+1] + shft] += delta_sp
              for e in edges[swp[j+1]]:
                if len(edges[e[0]]) > 0:
                  n_pherm[e[0] + shft] += delta_sp_hlf
          elif swp[j] >= 0:
            n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * last_drone_del] += delta
            if prev_tsj:
              n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
              n_pherm[swp[j] + NUM_NODES * cycle[cyc_ind]] += delta_sp
              for e in edges[swp[j]]:
                if len(edges[e[0]]) > 0:
                  n_pherm[e[0] + shft] += delta_sp_hlf
            else:
              sp_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
            if swp[j+1] >= 0:
              n_pherm[swp[j+1] + NUM_NODES * last_drone_del] += delta_sp
              for e in edges[swp[j+1]]:
                if len(edges[e[0]]) > 0:
                  n_pherm[e[0] + shft] += delta_sp_hlf
              last_drone_del = cycle[cyc_ind + 1]
            else:
              last_drone_del = None
            prev_tsj = False
          else:
            first_parent = cycle[cyc_ind + 1]
            last_drone_del = first_parent
            shft = NUM_NODES * cycle[cyc_ind]
            n_pherm[demand[cycle[cyc_ind + 1]][0] + shft] += delta
            n_pherm[swp[j+1] + shft] += delta_sp
            for e in edges[swp[j+1]]:
                if len(edges[e[0]]) > 0:
                  n_pherm[e[0] + shft] += delta_sp_hlf
          j += 2
          cyc_ind += 1
      # updating best cycle data
      _, cycle, swp = cycles[best_ind]
      for i in range(DEMAND_SIZE):
        best_cycle[i] = cycle[i]
      for i in range(SWP_SIZE):
        best_swp[i] = swp[i]
      j = 0
      while j < N_PHERM_SIZE:
        n_pherm[j] *= degradation_factor
        j += 1
      pbar.update()
      c = 1
      while c > 0:
        with saw_zero.get_lock():
          c = saw_zero.value
      with barrier.get_lock():
        barrier.value = ants_per_iter
    pbar.close()
    print("ACO complete!")
    for p in processes:
      p.join()
      p.close()
    return best_energy, best_cycle, best_swp

  def show_swp_string(self, swp, line_break_freq=50):
    lst1, lst2, lst3 = [], [], []
    l1, l2, l3 = "", "", ""
    spc = "   "
    ctr = "==="
    splitl = "=--"
    splitr = splitl[::-1]
    divup = " /`"
    divdn = " \\_"
    condn = "`\ "
    conup = "_/ "
    top = "```"
    bot = "___"
    prev_tsj = False
    if swp[0] >= 0:
      l1 += divup
      l2 += splitl
      l3 += spc
    else:
      l1 += spc
      l2 += ctr
      l3 += spc
    j = 1
    cyc_ind = 0
    while j < len(swp) - 1:
      if swp[j] == -2:
        if swp[j+1] == -2:
          if prev_tsj:
            l1 += top
            l2 += spc
            l3 += bot
          else:
            l1 += top
            l2 += "-  "
            l3 += divdn
            prev_tsj = True
        elif swp[j+1] == -1:
          l1 += spc
          l2 += ctr
          l3 += spc
        else:
          l1 += top
          l2 += "---"
          l3 += spc
      elif swp[j] >= 0:
        if swp[j+1] >= 0:
          if prev_tsj:
            l1 += "\\ /"
            l2 += " = "
            l3 += "/ \\"
            prev_tsj = False
          else:
            l1 += "`\\ /`"
            l2 += "--=--"
            l3 += spc
        else:
          if prev_tsj:
            l1 += condn
            l2 += "  ="
            l3 += conup
            prev_tsj = False
          else:
            l1 += condn
            l2 += splitr
            l3 += spc
      else:
        l1 += divup
        l2 += splitl
        l3 += spc
      j += 2
      if cyc_ind % line_break_freq == 0:
        lst1.append(l1)
        lst2.append(l2)
        lst3.append(l3)
        l1, l2, l3 = "", "", ""
      cyc_ind += 1
    if swp[j] >= 0:
      if prev_tsj:
        l1 += condn
        l2 += "  ="
        l3 += conup
      else:
        l1 += condn
        l2 += splitr
        l3 += spc
    else:
      l1 += spc
      l2 += ctr
      l3 += spc
    for i in range(len(lst1)):
      print(lst1[i])
      print(lst2[i])
      print(lst3[i])
    print(l1)
    print(l2)
    print(l3)

def construct_time(llep_d, start_ind, to_visit, edges, dedges, DS):
  """
  assumes no switch point for a demand is the demand node itself.
  """
  lim = len(to_visit) - 2
  time, tvs_ind, prv, cur, cur_id, cur_ty = 0, start_ind, -1, -1, -1, None
  lep_frm, lep_to, e = None, None, None
  while tvs_ind < lim:
    lep_frm, lep_to = llep_d[to_visit[tvs_ind + 1][0]]
    prv = to_visit[tvs_ind]
    cur, cur_ty = lep_to[prv]
    while cur != -1:
      if cur_ty:
        e = edges[prv][cur]
      else:
        e = dedges[prv][cur]
      time += e[1] / DS
      prv = cur
      cur, cur_ty = lep_to[cur]
    cur, cur_id, cur_ty = lep_frm[to_visit[tvs_ind + 2]]
    while cur != -1:
      if cur_ty:
        e = edges[cur][cur_id]
      else:
        e = dedges[cur][cur_id]
      time += e[1] / DS
      cur, cur_id, cur_ty = lep_frm[cur]
    tvs_ind += 2
  _, lep_to = llep_d[to_visit[tvs_ind + 1][0]]
  prv = to_visit[tvs_ind]
  cur, cur_ty = lep_to[prv]
  while cur != -1:
    if cur_ty:
      e = edges[prv][cur]
    else:
      e = dedges[prv][cur]
    time += e[1] / DS
    prv = cur
    cur, cur_ty = lep_to[cur]
  return time

def construct_time_truck(lep_t, from_node, to_node, edges):
  time = 0
  cur, cur_id = lep_t[from_node][to_node]
  while cur != -1:
    e = edges[cur][cur_id]
    time += e[1] / e[3]
    cur, cur_id = lep_t[from_node][cur]
  return time

def construct_energy(llep_d, to_visit, edges, dedges, dron_w, DS):
  lim, w_ind = len(to_visit) - 2, int(dron_w * 4)
  eng, tvs_ind, prv, cur, cur_id, cur_ty = ENG_ZERO, 0, -1, -1, -1, None
  lep_frm, lep_to, e = None, None, None
  while tvs_ind < lim:
    lep_frm, lep_to = llep_d[to_visit[tvs_ind + 1][0]]
    prv = to_visit[tvs_ind]
    cur, cur_ty = lep_to[prv]
    while cur != -1:
      if cur_ty:
        e = edges[prv][cur]
        eng += e[4][w_ind] * e[1] / DS
      else:
        e = dedges[prv][cur]
        eng += e[2][w_ind] * e[1] / DS
      prv = cur
      cur, cur_ty = lep_to[cur]
    cur, cur_id, cur_ty = lep_frm[to_visit[tvs_ind + 2]]
    while cur != -1:
      if cur_ty:
        e = edges[cur][cur_id]
        eng += e[4][w_ind] * e[1] / DS
      else:
        e = dedges[cur][cur_id]
        eng += e[2][w_ind] * e[1] / DS
      cur, cur_id, cur_ty = lep_frm[cur]
    tvs_ind += 2
  _, lep_to = llep_d[to_visit[tvs_ind + 1][0]]
  prv = to_visit[tvs_ind]
  cur, cur_ty = lep_to[prv]
  while cur != -1:
    if cur_ty:
      e = edges[prv][cur]
      eng += e[4][w_ind] * e[1] / DS
    else:
      e = dedges[prv][cur]
      eng += e[2][w_ind] * e[1] / DS
    prv = cur
    cur, cur_ty = lep_to[cur]
  return eng

def construct_energy_launch(llep_d, to_dem, sp, edges, dedges, dron_w, DS):
  eng, cur, cur_ty, w_ind = ENG_ZERO, -1, -1, int(dron_w * 4)
  _, lep_to = llep_d[to_dem]
  cur, cur_ty = lep_to[sp]
  while cur != -1:
    if cur_ty:
      e = edges[sp][cur]
      eng += e[2][w_ind] * e[1] / DS
    else:
      e = dedges[sp][cur]
      eng += e[2][w_ind] * e[1] / DS
    sp = cur
    cur, cur_ty = lep_to[cur]
  return eng

def construct_energy_spef(llep_d, from_dem, sp, to_dem, edges, dedges, dron_w, DS):
  eng, cur, cur_ty, cur_id, w_ind = ENG_ZERO, -1, -1, -1, -1, int(dron_w * 4) 
  lep_frm, _ = llep_d[from_dem]
  _, lep_to = llep_d[to_dem]
  prv = sp
  cur, cur_ty = lep_to[prv]
  while cur != -1:
    if cur_ty:
      e = edges[prv][cur]
      eng += e[2][w_ind] * e[1] / DS
    else:
      e = dedges[prv][cur]
      eng += e[2][w_ind] * e[1] / DS
    prv = cur
    cur, cur_ty = lep_to[cur]
  cur, cur_id, cur_ty = lep_frm[sp]
  while cur != -1:
    if cur_ty:
      e = edges[cur][cur_id]
      eng += e[2][w_ind] * e[1] / DS
    else:
      e = dedges[cur][cur_id]
      eng += e[2][w_ind] * e[1] / DS
    cur, cur_id, cur_ty = lep_frm[cur]
  return eng

def construct_energy_meetup(llep_d, from_dem, sp, edges, dedges, DS):
  eng, cur, cur_ty, cur_id = ENG_ZERO, -1, -1, -1, -1
  lep_frm, _ = llep_d[from_dem]
  cur, cur_id, cur_ty = lep_frm[sp]
  while cur != -1:
    if cur_ty:
      e = edges[cur][cur_id]
      eng += e[2][0] * e[1] / DS
    else:
      e = dedges[cur][cur_id]
      eng += e[2][0] * e[1] / DS
    cur, cur_id, cur_ty = lep_frm[cur]
  return eng

def _aco_worker(barrier, saw_zero, demand, sp_poss, n_pherm, sp_pherm, cycle, 
                llep_d, lep_t, swp, let_t, K, DS, edges, dedges, result):
  src = demand.pop()[0]     # DRONE WEIGHT NEEDS MANUAL SYNC
  ALPHA, BETA, MAX_WEIGHT, DRONE_WEIGHT, TW_DENM = 0.9, 1.5, WEIGHTS[-1], 12, 4535.9237
  N_PHERM_SIZE, DEMAND_SIZE, PCKG_BONUS_COEFF = int(len(n_pherm)), len(demand), 10**(-4)
  NUM_NODES = int(N_PHERM_SIZE / (DEMAND_SIZE + 1))
  N_PHERM_LAST = N_PHERM_SIZE - NUM_NODES
  TOTAL_WEIGHT = sum(pr[1] for pr in demand) + DRONE_WEIGHT
  # src_local_poss = sp_poss.pop()  # list of node indexes
  # src_local_paths = llep_d.pop()  # local best paths
  # src_lep_t = lep_t.pop()         # list of paths to all nodes
  # src_let_t = let_t.pop()         # list of path total energies
  got = [0 for _ in range(DEMAND_SIZE)]
  f_dem_ps, f_dem_ws = [i for i in range(DEMAND_SIZE)], [0 for _ in range(DEMAND_SIZE)]
  sp_poss_set, nbs, ws, not_got_chance = [], [], [], False
  for i in range(len(sp_poss)):
    sp_poss_set.append((set(sp_poss[i][0]),set(sp_poss[i][1])))
  eng_rem, eng_to_add, eng_at_sp, next_dem_node, curr_shft, lval, flag = 0, 0, 0, -1, -1, -1, -1
  # -----------------------------
  # State Holders
  # drone_loc, truck_loc are indexes of demand array.
  # if truck waiting at a switch point, sel_sp is set.
  # -----------------------------
  truck_loc, truck_loc_node, truck_w, w_coeff, parent_loc_node, sp = None, None, None, None, None, None
  drone_loc, drone_loc_node, drone_w, to_visit, to_visit_truck, tvs_ind_st = None, None, None, [], [], None
  next_dem, eng_tot, num_delvd, swp_ind, eng_acc, pot_dem, parent_loc = None, None, None, None, None, None, None
  w_coeff_oth, truck_w_og, best_eng, best_sp, best_eng_to_add, sel_sp = None, None, None, None, None, None
  time_taken, prev_t_eng, truck_w_og, truck_side_eng, truck_side_w_lost = None, None, None, None, None
  # -----------------------------
  while K > 0:
    # -----------------------------
    # Early Exit Signal
    # -----------------------------
    if n_pherm[0] < 0:
      break
    # -----------------------------
    # Initial State & Deployment
    # -----------------------------
    truck_w, eng_tot, num_delvd, sel_sp = TOTAL_WEIGHT, 0, 0, -1
    w_coeff, swp_ind = 1 - (TOTAL_WEIGHT / TW_DENM), 0
    drone_loc, drone_loc_node, drone_w, ENG_LEVL = -1, -1, 0, MAX_BATTERY_USE
    for j in range(DEMAND_SIZE):
      f_dem_ws[j] = (n_pherm[N_PHERM_LAST + demand[j][0]])**ALPHA / \
                    (let_t[src][demand[j][0]] / w_coeff)**BETA
      got[j] = 0
    next_dem = choices(f_dem_ps, weights=f_dem_ws)[0]
    got[next_dem] = 0
    cycle[num_delvd] = next_dem
    num_delvd += 1
    if demand[next_dem][1] <= MAX_WEIGHT:
      # sample a switch point, if delivery node itself then none.
      nbs, ws = [], []
      eng_rem = ENG_LEVL - MIN_MEETUP_BATTERY_REM
      for sp in sp_poss[next_dem][1]:
        eng_to_add = construct_energy_launch(llep_d, next_dem, sp, edges, dedges, demand[next_dem][1], DS)
        if eng_rem - eng_to_add > 0 and len(edges[ind]) > 0:
          nbs.append((sp, eng_to_add))
          ws.append((n_pherm[N_PHERM_LAST + sp])**ALPHA / (let_t[src][sp] / w_coeff)**BETA)
      if len(nbs) > 0:
        sel_sp, eng_to_add = choices(nbs, weights=ws)[0]
        if sel_sp == next_dem:
          sel_sp = -1
    if sel_sp == -1:
      # not going to be allocating a switch point.
      truck_loc, truck_loc_node = next_dem, demand[next_dem][0]
      eng_tot += let_t[src][truck_loc_node] / w_coeff
      truck_w -= demand[next_dem][1]
      swp[swp_ind] = -1
    else:
      # allocated a switch point.
      truck_loc, truck_loc_node, drone_loc, drone_loc_node = -1, -1, next_dem, demand[next_dem][0]
      drone_w = demand[next_dem][1]
      truck_w -= DRONE_WEIGHT + drone_w
      eng_tot += (let_t[src][sel_sp] / w_coeff)
      to_visit.append(sel_sp)
      to_visit.append((next_dem, eng_to_add))
      swp[swp_ind] = sel_sp
    w_coeff = 1 - (truck_w / TW_DENM)
    swp_ind += 1
    # -----------------------------
    while num_delvd < DEMAND_SIZE:
      if drone_loc > 0:
        # giving drone the option to continue delivering
        # length of to_visit, to_visit_truck must be 0.
        eng_acc, tvs_ind_st, time_passed, truck_w_og = 0, 2, 0, truck_w
        time_passed += construct_time(llep_d, 0, to_visit, edges, dedges, DRONE_GROUND_SPEED)
        nbs, ws = [], []
        prev_t_eng = 0
        parent_loc = to_visit[1][0]
        parent_loc_node = to_visit[0]
        for i in range(DEMAND_SIZE):
          if got[i] == 0:
            time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
            if time_passed - time_taken > 0:
              nbs.append((i, time_taken))
              ws.append((sp_pherm[i + DEMAND_SIZE * parent_loc])**ALPHA /
                        (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
        while len(nbs) > 0:
          # can move away from switch point for work!
          pot_dem, time_taken = choices(nbs, weights=ws)[0]
          truck_w -= demand[pot_dem][1]
          w_coeff = 1 - (truck_w / TW_DENM)
          to_visit_truck.append((pot_dem, 1, prev_t_eng + let_t[parent_loc_node][demand[pot_dem][0]]))
          got[pot_dem] = 1
          time_passed -= time_taken
          parent_loc, _, prev_t_eng = to_visit_truck[-1]
          parent_loc_node = demand[parent_loc][0]
          nbs, ws = [], []
          for i in range(DEMAND_SIZE):
            if got[i] == 0:
              time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
              if time_passed - time_taken > 0:
                nbs.append((i, time_taken))
                ws.append((sp_pherm[i + DEMAND_SIZE * parent_loc])**ALPHA /
                          (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
        while ENG_LEVL - eng_acc > 0:
          nbs, ws, to_visit_truck = [], [], []
          curr_shft = NUM_NODES * drone_loc
          for i in range(DEMAND_SIZE):
            if got[i] == 0 and drone_w + demand[i][1] < MAX_WEIGHT:
              common_sps = sp_poss_set[drone_loc][0] & sp_poss_set[i][1]
              if len(common_sps) > 0:
                eng_so_far = construct_energy(llep_d, to_visit, edges, dedges, drone_w + demand[i][1], DS)
                if eng_so_far > MAX_BATTERY_USE:
                  continue
                eng_rem = ENG_LEVL - eng_so_far - MIN_MEETUP_BATTERY_REM
                # lval = (n_pherm[demand[i][0] + curr_shft])**ALPHA / (eng_so_far)**BETA
                lval = 0
                for sp in common_sps:
                  eng_to_add = construct_energy_spef(llep_d, drone_loc, sp, i, edges, dedges, demand[i][1], DS)
                  if eng_rem - eng_to_add > 0:
                    nbs.append((i, sp, eng_so_far + eng_to_add))
                    ws.append(lval + ((n_pherm[sp + curr_shft])**ALPHA / (eng_to_add)**BETA))
          if len(nbs) == 0:
            break
          drone_loc, sp, eng_acc = choices(nbs, weights=ws)[0]
          got[drone_loc] = 1
          drone_w += demand[drone_loc][1]
          truck_w -= demand[drone_loc][1]
          w_coeff = 1 - (truck_w / TW_DENM)
          to_visit.append(sp)
          to_visit.append((drone_loc, eng_acc))
          time_passed += construct_time(llep_d, tvs_ind_st, to_visit, edges, dedges, DRONE_GROUND_SPEED)
          nbs, ws = [], []
          if len(to_visit_truck) == 0:
            prev_t_eng = 0
            parent_loc = to_visit[1][0]
            parent_loc_node = to_visit[0]
          else:
            parent_loc, _, prev_t_eng = to_visit_truck[-1]
            parent_loc_node = demand[parent_loc][0]
          for i in range(DEMAND_SIZE):
            if got[i] == 0:
              time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
              if time_passed - time_taken > 0:
                nbs.append((i, time_taken))
                ws.append((sp_pherm[i + DEMAND_SIZE * parent_loc])**ALPHA /
                          (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
          while len(nbs) > 0:
            # can move away from switch point for work!
            pot_dem, time_taken = choices(nbs, weights=ws)[0]
            truck_w -= demand[pot_dem][1]
            w_coeff = 1 - (truck_w / TW_DENM)
            to_visit_truck.append((pot_dem, len(to_visit) - 1, prev_t_eng + let_t[parent_loc_node][demand[pot_dem][0]]))
            got[pot_dem] = 1
            time_passed -= time_taken
            parent_loc, _, prev_t_eng = to_visit_truck[-1]
            parent_loc_node = demand[parent_loc][0]
            nbs, ws = [], []
            for i in range(DEMAND_SIZE):
              if got[i] == 0:
                time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
                if time_passed - time_taken > 0:
                  nbs.append((i, time_taken))
                  ws.append((sp_pherm[i + DEMAND_SIZE * parent_loc])**ALPHA /
                            (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
          tvs_ind_st = len(to_visit)
        nbs, ws = [], []
        truck_w = truck_w_og
        tvs_ind_st, truck_side_eng, truck_side_w_lost = 0, 0, -1
        parent_loc, parent_loc_node = to_visit[1][0], demand[to_visit[0]]
        phm_src = sp_pherm, phm_src_shft = DEMAND_SIZE
        for i in range(1, len(to_visit) - 1, 2):
          drone_loc = to_visit[i][0]
          # catching up the truck!
          truck_side_w_lost = 0
          while tvs_ind_st < len(to_visit_truck) and to_visit_truck[tvs_ind_st][1] <= i:
            truck_side_eng = to_visit_truck[tvs_ind_st][2]
            parent_loc = to_visit_truck[tvs_ind_st][0]
            parent_loc_node = demand[parent_loc][0]
            truck_side_w_lost += demand[parent_loc][1]
            tvs_ind_st += 1
            if phm_src == n_pherm:
              continue
            phm_src = n_pherm
            phm_src_shft = NUM_NODES
          eng_rem = ENG_LEVL - to_visit[i][1]
          truck_w -= truck_side_w_lost + demand[drone_loc][1]
          pot_dem = demand[to_visit[i + 2][0]][0]   # now holds node
          w_coeff = 1 - (truck_w / TW_DENM)
          w_coeff_oth = 1 - ((truck_w + DRONE_WEIGHT) / TW_DENM)
          # to meetup with drone, find optimal
          # switch point to minimize total
          # consumption, phermone free !
          best_eng, best_sp = float('inf'), -1
          for ind in sp_poss[drone_loc][0]:
            eng_to_add = construct_energy_meetup(llep_d, drone_loc, ind, edges, dedges, DS)
            if eng_rem - eng_to_add > 0 and len(edges[ind]) > 0:
              tmp = eng_to_add + (let_t[parent_loc_node][ind] / w_coeff) + (let_t[ind][pot_dem] / w_coeff_oth)
              if tmp < best_eng:
                best_eng = tmp
                best_sp = ind
                best_eng_to_add = eng_to_add
          if best_eng == float('inf'):   # drone can't move at all.
            best_sp = drone_loc
            best_eng_to_add = ENG_ZERO
          nbs.append((i + 2, -2, best_sp, best_eng_to_add))
          ws.append(((phm_src[pot_dem + phm_src_shft * parent_loc] + 
                      (truck_w_og - truck_w) * PCKG_BONUS_COEFF)**ALPHA / 
                     ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                      (let_t[best_sp][pot_dem] / w_coeff_oth) +
                      best_eng_to_add + truck_side_eng)**BETA))
        # all side jobs for last drone delivery
        truck_side_w_lost = 0
        while tvs_ind_st < len(to_visit_truck):
            truck_side_eng = to_visit_truck[tvs_ind_st][2]
            parent_loc = to_visit_truck[tvs_ind_st][0]
            parent_loc_node = demand[parent_loc][0]
            truck_side_w_lost += demand[parent_loc][1]
            tvs_ind_st += 1
            if phm_src == n_pherm:
              continue
            phm_src = n_pherm
            phm_src_shft = NUM_NODES
        truck_w -= truck_side_w_lost
        drone_loc = to_visit[-1][0]
        eng_rem = ENG_LEVL - to_visit[-1][1]
        w_coeff = 1 - (truck_w / TW_DENM)
        w_coeff_oth = 1 - ((truck_w + DRONE_WEIGHT) / TW_DENM)
        if (len(to_visit) / 2) + len(to_visit_truck) + num_delvd >= DEMAND_SIZE: # if none remaining, allow src to be last
          pot_dem = src
          best_eng, best_sp = float('inf'), -1
          for ind in sp_poss[drone_loc][0]:
            eng_to_add = construct_energy_meetup(llep_d, drone_loc, ind, edges, dedges, DS)
            if eng_rem - eng_to_add > 0 and len(edges[ind]) > 0:
              tmp = eng_to_add + (let_t[parent_loc_node][ind] / w_coeff) + (let_t[ind][pot_dem] / w_coeff_oth)
              if tmp < best_eng:
                best_eng = tmp
                best_sp = ind
                best_eng_to_add = eng_to_add
          if best_eng == float('inf'):   # drone can't move at all.
            best_sp = drone_loc
            best_eng_to_add = 0
          nbs.append((len(to_visit) + 1, -1, best_sp, best_eng_to_add))
          ws.append(((phm_src[pot_dem + phm_src_shft * parent_loc] + 
                      (truck_w_og - truck_w) * PCKG_BONUS_COEFF +
                      n_pherm[pot_dem + NUM_NODES * drone_loc])**ALPHA / 
                     ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                      (let_t[best_sp][pot_dem] / w_coeff_oth) +
                      best_eng_to_add + truck_side_eng)**BETA))
        else:   # otherwise choose a new demand to consider post drone operation.
          for i in range(DEMAND_SIZE):
            if got[i] == 0:
              best_eng, best_sp = float('inf'), -1
              for ind in sp_poss[drone_loc][0]:
                eng_to_add = construct_energy_meetup(llep_d, drone_loc, ind, edges, dedges, DS)
                if eng_rem - eng_to_add > 0 and len(edges[ind]) > 0:
                  tmp = eng_to_add + (let_t[parent_loc_node][ind] / w_coeff) + (let_t[ind][demand[i][0]] / w_coeff_oth)
                  if tmp < best_eng:
                    best_eng = tmp
                    best_sp = ind
                    best_eng_to_add = eng_to_add
              if best_eng == float('inf'):   # drone can't move at all.
                best_sp = drone_loc
                best_eng_to_add = ENG_ZERO
              nbs.append((len(to_visit) + 1, i, best_sp, best_eng_to_add))
              ws.append(((phm_src[pot_dem + phm_src_shft * parent_loc] + 
                          (truck_w_og - truck_w) * PCKG_BONUS_COEFF +
                          n_pherm[demand[i][0] + NUM_NODES * drone_loc])**ALPHA / 
                         ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                          (let_t[best_sp][pot_dem] / w_coeff_oth) +
                          best_eng_to_add + truck_side_eng)**BETA))
        ind, flag, sp, best_eng_to_add = choices(nbs, weights=ws)[0]
        truck_w = truck_w_og + DRONE_WEIGHT
        parent_loc, parent_loc_node = to_visit[1][0], demand[to_visit[0]]
        # adding selected demand after end point confirmed
        # first drone itenary
        for i in range(3, ind, 2):
          drone_loc = to_visit[i][0]
          swp[swp_ind] = -2
          swp[swp_ind + 1] = to_visit[i - 1]
          swp_ind += 2
          cycle[num_delvd] = drone_loc
          num_delvd += 1
          truck_w -= demand[drone_loc][1]
        # then truck itenary
        tvs_ind_st = 0
        for i in range(3, ind, 2):
          while tvs_ind_st < len(to_visit_truck) and to_visit_truck[tvs_ind_st][1] <= i:
            swp[swp_ind] = -2
            swp[swp_ind + 1] = -2
            swp_ind += 2
            parent_loc = to_visit_truck[tvs_ind_st][0]
            parent_loc_node = demand[parent_loc][0]
            cycle[num_delvd] = parent_loc
            num_delvd += 1
            truck_w -= demand[parent_loc][1]
            tvs_ind_st += 1
        if len(tvs_ind_st) > 0:
          if tvs_ind_st > 0:
            # one or more demands on the side completed
            eng_tot += to_visit_truck[tvs_ind_st - 1][2]
          while tvs_ind_st < len(to_visit_truck):
            got[to_visit_truck[tvs_ind_st][0]] = 0
        for i in range(ind + 2, len(to_visit), 2):
          got[to_visit[i][0]] = 0
        w_coeff = 1 - (truck_w / TW_DENM)
        ENG_LEVL -= to_visit[ind - 2][1] + best_eng_to_add
        # --------------------------------------
        # Drone location must be with drone_loc, 
        # next_loc must point to next demand,
        # and sel_sp is the switch point node. 
        # --------------------------------------
        eng_tot += to_visit[ind - 2][1] + best_eng_to_add + let_t[parent_loc_node][sp]
        sel_sp = sp
        swp[swp_ind] = sp
        swp_ind += 1
        drone_loc = to_visit[ind - 2][0]
        if flag == -2:
          next_dem = to_visit[ind][0]
        elif flag >= 0:
          next_dem = flag
          got[next_dem] = 1
        else:
          break
        cycle[num_delvd] = next_dem
        num_delvd += 1
        to_visit, to_visit_truck = [], []
        # drone just came back to the truck at switch point.
        truck_loc_node = sel_sp  #  holds current selected switch point.
        if demand[next_dem][1] <= MAX_WEIGHT:
          nbs, ws = [], []
          for sp in sp_poss[next_dem][1]:
            eng_at_sp = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, sp, edges) * RECARGE_RATE)
            eng_to_add = construct_energy_launch(llep_d, next_dem, sp, edges, dedges, demand[next_dem][1], DS)
            if eng_at_sp - eng_to_add > MIN_MEETUP_BATTERY_REM:
              nbs.append((sp, eng_to_add, eng_at_sp))
              ws.append((n_pherm[sp + NUM_NODES * drone_loc])**ALPHA / 
                        (let_t[truck_loc_node][sp] / w_coeff)**BETA)
          if len(nbs) > 0:
            sel_sp, eng_to_add, eng_at_sp = choices(nbs, weights=ws)[0]
            if sel_sp == next_dem:
              sel_sp = -1
        if sel_sp == -1:
          next_dem_node = demand[next_dem][0]
          eng_tot += let_t[truck_loc_node][next_dem_node] / w_coeff
          truck_w -= demand[next_dem][1]
          ENG_LEVL = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, next_dem_node, edges) * RECARGE_RATE)
          truck_loc, truck_loc_node, drone_loc, drone_loc_node = next_dem, next_dem_node, -1, -1
          swp[swp_ind] = -1
        else:
          # allocated a switch point.
          eng_tot += (let_t[truck_loc_node][sel_sp] / w_coeff)
          ENG_LEVL = eng_at_sp
          truck_loc, truck_loc_node, drone_loc, drone_loc_node = -1, -1, next_dem, demand[next_dem][0]
          drone_w = demand[next_dem][1]
          truck_w -= DRONE_WEIGHT + drone_w
          to_visit.append(sel_sp)
          to_visit.append((next_dem, eng_to_add))
          swp[swp_ind] = sel_sp
        w_coeff = 1 - (truck_w / TW_DENM)
        swp_ind += 1
      else:
        # letting truck continue and decide whether to allocate a switch point.
        nbs, ws = [], []
        for i in range(DEMAND_SIZE):
          if got[i] == 0:
            nbs.append(i)
            ws.append(((n_pherm[demand[i][0] + NUM_NODES * truck_loc])**ALPHA / 
                       (let_t[truck_loc_node][demand[i][0]] / w_coeff)**BETA))
        next_dem = choices(nbs, weights=ws)[0]
        got[next_dem] = 0
        cycle[num_delvd] = next_dem
        num_delvd += 1
        if demand[next_dem] <= MAX_WEIGHT:
          # sample a switch point, if delivery node itself then none.
          nbs, ws = [], []
          for sp in sp_poss[next_dem][1]:
            eng_at_sp = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, sp, edges) * RECARGE_RATE)
            eng_to_add = construct_energy_launch(llep_d, next_dem, sp, edges, dedges, demand[next_dem][1], DS)
            if eng_at_sp - eng_to_add > MIN_MEETUP_BATTERY_REM:
              nbs.append((sp, eng_to_add, eng_at_sp))
              ws.append((n_pherm[truck_loc * NUM_NODES + sp])**ALPHA / 
                        (let_t[truck_loc_node][sp] / w_coeff)**BETA)
          if len(nbs) > 0:
            sel_sp, eng_to_add, eng_at_sp = choices(nbs, weights=ws)[0]
            if sel_sp == next_dem:
              sel_sp = -1
        if sel_sp == -1:
          # not going to be allocating a switch point.
          next_dem_node = demand[next_dem][0]
          eng_tot += let_t[truck_loc_node][next_dem_node] / w_coeff
          ENG_LEVL = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, next_dem_node, edges) * RECARGE_RATE)
          truck_loc, truck_loc_node = next_dem, next_dem_node
          truck_w -= demand[next_dem][1]
          swp[swp_ind] = -2
          swp[swp_ind + 1] = -1
        else:
          # allocated a switch point.
          eng_tot += (let_t[truck_loc_node][sel_sp] / w_coeff)
          ENG_LEVL = eng_at_sp
          truck_loc, truck_loc_node, drone_loc, drone_loc_node = -1, -1, next_dem, demand[next_dem][0]
          drone_w = demand[next_dem][1]
          truck_w -= DRONE_WEIGHT + drone_w
          to_visit.append(sel_sp)
          to_visit.append((next_dem, eng_to_add))
          swp[swp_ind] = -1
          swp[swp_ind + 1] = sel_sp
        w_coeff = 1 - (truck_w / TW_DENM)
        swp_ind += 2
    # -----------------------------
    # Final Deployment
    # -----------------------------
    assert abs(truck_w - DRONE_WEIGHT) < 0.1, "WEIGHT NOT GOOD"
    if sel_sp == -1:
      eng_tot += let_t[sel_sp][src] / w_coeff
      swp[swp_ind] = sel_sp
    else:
      eng_tot += let_t[truck_loc_node][src] / w_coeff
      swp[swp_ind] = -1
    result.value = eng_tot
    K -= 1
    # -----------------------------
    # Synchronizer
    # -----------------------------
    with barrier.get_lock():
      barrier.value -= 1
    not_got_chance = True
    while not_got_chance:
      with barrier.get_lock():
        if barrier.value == 0:
          not_got_chance = False
    with saw_zero.get_lock():
      saw_zero.value -= 1
    not_got_chance = True
    while not_got_chance:
      with barrier.get_lock():
        if barrier.value > 0:
          not_got_chance = False
    # -----------------------------

def _aco_worker_truck_only(barrier, saw_zero, demand, n_pherm, cycle, let_t, K, result):
  src = demand.pop()[0]     # DRONE WEIGHT NEEDS MANUAL SYNC
  ALPHA, BETA, TW_DENM = 0.9, 1.5, 4535.9237
  N_PHERM_SIZE, DEMAND_SIZE = int(len(n_pherm)), len(demand), 10**(-4)
  NUM_NODES = int(N_PHERM_SIZE / (DEMAND_SIZE + 1))
  N_PHERM_LAST = N_PHERM_SIZE - NUM_NODES
  TOTAL_WEIGHT = sum(pr[1] for pr in demand)
  # src_local_poss = sp_poss.pop()  # list of node indexes
  # src_local_paths = llep_d.pop()  # local best paths
  # src_lep_t = lep_t.pop()         # list of paths to all nodes
  # src_let_t = let_t.pop()         # list of path total energies
  got = [0 for _ in range(DEMAND_SIZE)]
  f_dem_ps, f_dem_ws = [i for i in range(DEMAND_SIZE)], [0 for _ in range(DEMAND_SIZE)]
  nbs, ws, not_got_chance = [], [], False
  # -----------------------------
  # State Holders
  # drone_loc, truck_loc are indexes of demand array.
  # if truck waiting at a switch point, sel_sp is set.
  # -----------------------------
  truck_loc, truck_loc_node, truck_w, w_coeff = None, None, None, None
  next_dem, next_dem_node, eng_tot, num_delvd = None, None, None, None
  # -----------------------------
  while K > 0:
    # -----------------------------
    # Early Exit Signal
    # -----------------------------
    if n_pherm[0] < 0:
      break
    # -----------------------------
    # Initial State & Deployment
    # -----------------------------
    truck_w, eng_tot, num_delvd = TOTAL_WEIGHT, 0, 0
    w_coeff = 1 - (TOTAL_WEIGHT / TW_DENM)
    for j in range(DEMAND_SIZE):
      f_dem_ws[j] = (n_pherm[N_PHERM_LAST + demand[j][0]])**ALPHA / \
                    (let_t[src][demand[j][0]] / w_coeff)**BETA
      got[j] = 0
    next_dem = choices(f_dem_ps, weights=f_dem_ws)[0]
    got[next_dem] = 0
    cycle[num_delvd] = next_dem
    num_delvd += 1
    eng_tot += let_t[src][truck_loc_node] / w_coeff
    truck_loc, truck_loc_node = next_dem, demand[next_dem][0]
    truck_w -= demand[next_dem][1]
    w_coeff = 1 - (truck_w / TW_DENM)
    # -----------------------------
    while num_delvd < DEMAND_SIZE:
      # letting truck continue and decide whether to allocate a switch point.
      nbs, ws = [], []
      for i in range(DEMAND_SIZE):
        if got[i] == 0:
          nbs.append(i)
          ws.append(((n_pherm[demand[i][0] + NUM_NODES * truck_loc])**ALPHA / 
                     (let_t[truck_loc_node][demand[i][0]] / w_coeff)**BETA))
      next_dem = choices(nbs, weights=ws)[0]
      got[next_dem] = 0
      cycle[num_delvd] = next_dem
      num_delvd += 1
      next_dem_node = demand[next_dem][0]
      eng_tot += let_t[truck_loc_node][next_dem_node] / w_coeff
      truck_loc, truck_loc_node = next_dem, next_dem_node
      truck_w -= demand[next_dem][1]
      w_coeff = 1 - (truck_w / TW_DENM)
    # -----------------------------
    # Final Deployment
    # -----------------------------
    assert abs(truck_w - DRONE_WEIGHT) < 0.1, "WEIGHT NOT GOOD"
    result.value = eng_tot + let_t[truck_loc_node][src] / w_coeff
    K -= 1
    # -----------------------------
    # Synchronizer
    # -----------------------------
    with barrier.get_lock():
      barrier.value -= 1
    not_got_chance = True
    while not_got_chance:
      with barrier.get_lock():
        if barrier.value == 0:
          not_got_chance = False
    with saw_zero.get_lock():
      saw_zero.value -= 1
    not_got_chance = True
    while not_got_chance:
      with barrier.get_lock():
        if barrier.value > 0:
          not_got_chance = False
    # -----------------------------

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
  W += DRONE_WEIGHT
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
              initializer=copy_globals_energy,
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
    edge_work[i] = (u_ind, ind, p.apply_async(_energy_worker, (rho, V_w_hd, V_w_lt), callback=update))
  for i in range(len(dedge_work)):
    u_ind, ind, rho, V_w_hd, V_w_lt = dedge_work[i]
    dedge_work[i] = (u_ind, ind, p.apply_async(_energy_worker, (rho, V_w_hd, V_w_lt), callback=update))
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

def _energy_worker(rho, V_w_hd, V_w_lt):
  drone_power = []
  for w in WEIGHTS:
    drone_power.append(power(rho, w, V_w_hd, V_w_lt))
  return tuple(drone_power)
