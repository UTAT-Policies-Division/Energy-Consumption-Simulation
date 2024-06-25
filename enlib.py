from math import sqrt, acos, atan2, pi, \
                 sin, cos, tan, exp, \
                 floor, ceil, log10
from random import randint, sample
import matplotlib.pyplot as plt
import pickle
import multiprocessing as mp
from os import cpu_count
from tqdm import tqdm
from PIL import Image
from workers import _aco_worker, _aco_worker_truck_only
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
BATTERY_CAPACITY = 17.0 * 42 * 3600 # J
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
    self.sji_pherm = None
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

  def get_node_to_dem_ind_map(self):
   dct = {}
   for i in range(len(self.demand)):
     dct[self.demand[i][0]] = i
   return dct

  def plot_cycle(self, K, cycle, swp=None, save=False, pic_number=0):
    if swp == None:
      swp = [-1 for _ in range(2 * len(self.demand))]
      for j in range(1, len(swp), 2):
        swp[j] = -2
    src = self.demand.pop()[0]
    # node_to_dem_ind = self.get_node_to_dem_ind_map()
    drone_color, drone_alpha, drone_width = "blue", 0.45, 2
    truck_color, truck_alpha, truck_width = "black", 0.4, 3
    limit, j, cyc_ind, px, py, dx, dy = 2 * len(self.demand) - 1, 1, 0, 0, 0, 0, 0
    truck_x, truck_y, drone_x, drone_y, drone_xls, drone_yls = [], [], [], [], [], []
    drone_loc, truck_loc_node, next_loc_node = None, None, None
    if swp[0] >= 0:
      lep_to = self.llep_d[cycle[0]][1]
      prv = swp[0]
      drone_x.append(self.nodes[prv][0])
      drone_y.append(self.nodes[prv][1])
      cur, cur_ty = lep_to[prv]
      while cur != -1:
        if cur_ty:
          e = self.edges[prv][cur]
        else:
          e = self.dedges[prv][cur]
        prv = e[0]
        drone_x.append(self.nodes[prv][0])
        drone_y.append(self.nodes[prv][1])
        cur, cur_ty = lep_to[prv]
      next_loc_node = swp[0]
    else:
      next_loc_node = self.demand[cycle[0]][0]
    tmp_x, tmp_y = [], []
    tmp_x.append(self.nodes[next_loc_node][0])
    tmp_y.append(self.nodes[next_loc_node][1])
    cur, _ = self.lep_t[src][next_loc_node]
    while cur != -1:
      tmp_x.append(self.nodes[cur][0])
      tmp_y.append(self.nodes[cur][1])
      cur, _ = self.lep_t[src][cur]
    tmp_x.reverse()
    tmp_y.reverse()
    truck_x.extend(tmp_x)
    truck_y.extend(tmp_y)
    truck_loc_node = next_loc_node
    drone_loc = cycle[0]
    while j < limit:
      if swp[j] == -2:
        if swp[j+1] == -1:
          tmp_x, tmp_y = [], []
          next_loc_node = self.demand[cycle[cyc_ind + 1]][0]
          tmp_x.append(self.nodes[next_loc_node][0])
          tmp_y.append(self.nodes[next_loc_node][1])
          cur, _ = self.lep_t[truck_loc_node][next_loc_node]
          while cur != -1:
            tmp_x.append(self.nodes[cur][0])
            tmp_y.append(self.nodes[cur][1])
            cur, _ = self.lep_t[truck_loc_node][cur]
          tmp_x.reverse()
          tmp_y.reverse()
          truck_x.extend(tmp_x)
          truck_y.extend(tmp_y)
          truck_loc_node = next_loc_node
          drone_loc = cycle[cyc_ind + 1]
        elif swp[j+1] == -2:
          tmp_x, tmp_y = [], []
          next_loc_node = self.demand[cycle[cyc_ind + 1]][0]
          tmp_x.append(self.nodes[next_loc_node][0])
          tmp_y.append(self.nodes[next_loc_node][1])
          cur, _ = self.lep_t[truck_loc_node][next_loc_node]
          while cur != -1:
            tmp_x.append(self.nodes[cur][0])
            tmp_y.append(self.nodes[cur][1])
            cur, _ = self.lep_t[truck_loc_node][cur]
          tmp_x.reverse()
          tmp_y.reverse()
          truck_x.extend(tmp_x)
          truck_y.extend(tmp_y)
          truck_loc_node = next_loc_node
        else:
          lep_frm = self.llep_d[drone_loc][0]
          next_loc_node = self.demand[cycle[cyc_ind + 1]]
          lep_to = self.llep_d[cycle[cyc_ind + 1]][1]
          tmp_x, tmp_y = [], []
          tmp_x.append(self.nodes[swp[j+1]][0])
          tmp_y.append(self.nodes[swp[j+1]][1])
          cur, _, _ = lep_frm[swp[j+1]]
          while cur != -1:
            tmp_x.append(self.nodes[cur][0])
            tmp_y.append(self.nodes[cur][1])
            cur, _, _ = lep_frm[cur]
          tmp_x.reverse()
          tmp_y.reverse()
          drone_x.extend(tmp_x)
          drone_y.extend(tmp_y)
          prv = swp[j+1]
          cur, cur_ty = lep_to[prv]
          while cur != -1:
            if cur_ty:
              e = self.edges[prv][cur]
            else:
              e = self.dedges[prv][cur]
            prv = e[0]
            drone_x.append(self.nodes[prv][0])
            drone_y.append(self.nodes[prv][1])
            cur, cur_ty = lep_to[prv]
          drone_loc = cycle[cyc_ind + 1]
      elif swp[j] >= 0:
        lep_frm = self.llep_d[drone_loc][0]
        tmp_x, tmp_y = [], []
        tmp_x.append(self.nodes[swp[j]][0])
        tmp_y.append(self.nodes[swp[j]][1])
        cur, _, _ = lep_frm[swp[j]]
        while cur != -1:
          tmp_x.append(self.nodes[cur][0])
          tmp_y.append(self.nodes[cur][1])
          cur, _, _ = lep_frm[cur]
        tmp_x.reverse()
        tmp_y.reverse()
        drone_x.extend(tmp_x)
        drone_y.extend(tmp_y)
        drone_xls.append(drone_x)
        drone_yls.append(drone_y)
        drone_x, drone_y = [], []
        tmp_x, tmp_y = [], []
        tmp_x.append(self.nodes[swp[j]][0])
        tmp_y.append(self.nodes[swp[j]][1])
        cur, _ = self.lep_t[truck_loc_node][swp[j]]
        while cur != -1:
          tmp_x.append(self.nodes[cur][0])
          tmp_y.append(self.nodes[cur][1])
          cur, _ = self.lep_t[truck_loc_node][cur]
        tmp_x.reverse()
        tmp_y.reverse()
        truck_x.extend(tmp_x)
        truck_y.extend(tmp_y)
        if swp[j+1] >= 0:
          lep_to = self.llep_d[cycle[cyc_ind + 1]][1]
          prv = swp[j+1]
          drone_x.append(self.nodes[prv][0])
          drone_y.append(self.nodes[prv][1])
          cur, cur_ty = lep_to[prv]
          while cur != -1:
            if cur_ty:
              e = self.edges[prv][cur]
            else:
              e = self.dedges[prv][cur]
            prv = e[0]
            drone_x.append(self.nodes[prv][0])
            drone_y.append(self.nodes[prv][1])
            cur, cur_ty = lep_to[prv]
          next_loc_node = swp[j+1]
        else:
          next_loc_node = self.demand[cycle[cyc_ind + 1]][0]
        tmp_x, tmp_y = [], []
        cur, _ = self.lep_t[swp[j]][next_loc_node]
        tmp_x.append(self.nodes[next_loc_node][0])
        tmp_y.append(self.nodes[next_loc_node][1])
        while cur != -1:
          tmp_x.append(self.nodes[cur][0])
          tmp_y.append(self.nodes[cur][1])
          cur, _ = self.lep_t[swp[j]][cur]
        tmp_x.reverse()
        tmp_y.reverse()
        truck_x.extend(tmp_x)
        truck_y.extend(tmp_y)
        truck_loc_node = next_loc_node
        drone_loc = cycle[cyc_ind + 1]
      else:
        tmp_x, tmp_y = [], []
        tmp_x.append(self.nodes[swp[j+1]][0])
        tmp_y.append(self.nodes[swp[j+1]][1])
        cur, _ = self.lep_t[truck_loc_node][swp[j+1]]
        while cur != -1:
          tmp_x.append(self.nodes[cur][0])
          tmp_y.append(self.nodes[cur][1])
          cur, _ = self.lep_t[truck_loc_node][cur]
        tmp_x.reverse()
        tmp_y.reverse()
        truck_x.extend(tmp_x)
        truck_y.extend(tmp_y)
        lep_to = self.llep_d[cycle[cyc_ind + 1]][1]
        prv = swp[j+1]
        cur, cur_ty = lep_to[prv]
        while cur != -1:
          if cur_ty:
            e = self.edges[prv][cur]
          else:
            e = self.dedges[prv][cur]
          prv = e[0]
          drone_x.append(self.nodes[prv][0])
          drone_y.append(self.nodes[prv][1])
          cur, cur_ty = lep_to[prv]
        drone_loc = cycle[cyc_ind + 1]
        truck_loc_node = swp[j+1]
      j += 2
      cyc_ind += 1
    if swp[j] >= 0:
      lep_frm = self.llep_d[drone_loc][0]
      tmp_x, tmp_y = [], []
      tmp_x.append(self.nodes[swp[j]][0])
      tmp_y.append(self.nodes[swp[j]][1])
      cur, _, _ = lep_frm[swp[j]]
      while cur != -1:
        tmp_x.append(self.nodes[cur][0])
        tmp_y.append(self.nodes[cur][1])
        cur, _, _ = lep_frm[cur]
      tmp_x.reverse()
      tmp_y.reverse()
      drone_x.extend(tmp_x)
      drone_y.extend(tmp_y)
      tmp_x, tmp_y = [], []
      tmp_x.append(self.nodes[swp[j]][0])
      tmp_y.append(self.nodes[swp[j]][1])
      cur, _ = self.lep_t[truck_loc_node][swp[j]]
      while cur != -1:
        tmp_x.append(self.nodes[cur][0])
        tmp_y.append(self.nodes[cur][1])
        cur, _ = self.lep_t[truck_loc_node][cur]
      tmp_x.reverse()
      tmp_y.reverse()
      truck_x.extend(tmp_x)
      truck_y.extend(tmp_y)
      truck_loc_node = swp[j]
    tmp_x, tmp_y = [], []
    tmp_x.append(self.nodes[src][0])
    tmp_y.append(self.nodes[src][1])
    cur, _ = self.lep_t[truck_loc_node][src]
    while cur != -1:
      tmp_x.append(self.nodes[cur][0])
      tmp_y.append(self.nodes[cur][1])
      cur, _ = self.lep_t[truck_loc_node][cur]
    tmp_x.reverse()
    tmp_y.reverse()
    truck_x.extend(tmp_x)
    truck_y.extend(tmp_y)
    if len(drone_x) > 0:
      drone_xls.append(drone_x)
      drone_yls.append(drone_y)
    for i in range(len(drone_xls)):
      px = drone_xls[i][0]
      py = drone_yls[i][0]
      for j in range(1, len(drone_xls[i])):
        dx = drone_xls[i][j] - px
        dy = drone_yls[i][j] - py
        plt.arrow(px, py, dx, dy,
                  width=drone_width,
                  fc=drone_color,
                  ec=drone_color,
                  alpha=drone_alpha,
                  head_width=13,
                  length_includes_head=True)
        px = drone_xls[i][j]
        py = drone_yls[i][j]
    px = truck_x[0]
    py = truck_y[0]
    for j in range(1, len(truck_x)):
      dx = truck_x[j] - px
      dy = truck_y[j] - py
      plt.arrow(px, py, dx, dy,
                width=truck_width,
                fc=truck_color,
                ec=truck_color,
                alpha=truck_alpha,
                head_width=15,
                length_includes_head=True)
      px = truck_x[j]
      py = truck_y[j]
    got = [0 for _ in range(len(self.nodes))]
    nx, ny, nc = [], [], []
    dx, dy = [], []
    for p in self.demand:
      got[p[0]] = 1
      dx.append(self.nodes[p[0]][0])
      dy.append(self.nodes[p[0]][1])
    if self.n_pherm is not None and len(self.demand) > 0:
      comp = 0
      for i in range(len(self.nodes)):
        if got[i] == 1:
          continue
        nx.append(self.nodes[i][0])
        ny.append(self.nodes[i][1])
        if len(self.edges[i]) > 0:
          comp = 0.7 - min(0.3 * K * sum(self.n_pherm[j][i] for j in range(len(self.demand) + 1)), 0.7)
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
    plt.scatter(dx, dy, c="magenta", s=12, alpha=0.8)
    plt.scatter(x=nx, y=ny, color=nc, s=5, alpha=0.8)
    llx, lly, lln = self.line_cover
    for i in range(len(llx)):
      plt.plot(llx[i], lly[i], marker="", c="red", alpha=0.15)
    llx, lly, lln = self.line_cover_d
    for i in range(len(llx)):
      plt.plot(llx[i], lly[i], marker="", c="limegreen", alpha=0.2)
    self.demand.append((src, 0))
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

  def save_light(self, filename='network_data_light.pkl'):
    print("Saving Energy Helper (light) object...")
    output = open(filename, 'wb')
    self.line_cover = None
    self.line_cover_d = None
    self.sp_poss = None
    self.llep_d = None
    self.lep_t = None
    self.let_t = None
    pickle.dump(self, output, 2)
    output.close()
    print("Energy Helper (light) object saved!")

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
    ehobj.sji_pherm = obj.sji_pherm
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
    NODE_BASE_PHERM = 10**(-4)
    SP_PHERM_COEFF = 10**(-3)
    DEMAND_PHERM_ADV_COEFF = 25
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
            # if cur in lep2:
            #   break
        if i == demand[j][0]:
          sp_poss[j][0].append(i)
          lep2 = llep_d[j][0]
          for k1 in range(len(nodes)):
            if lep[k1][0] != -1:
              sp_poss[j][0].append(k1)
              n_pherm[j][k1] += (1 - abs(1 - (let[k1] / MAX_BATTERY_USE_HALF))) * SP_PHERM_COEFF
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
    max_phm, tmp = 0, 0
    for i in range(len(demand)):
      for j in range(len(demand)):
        if let_t[self.demand[i][0]][self.demand[j][0]] == float("inf"):
          print("BAD CONNECTION FOUND:", self.demand[i][0], self.demand[j][0], ". Aborting...")
          exit(1)
        tmp = DEMAND_BASE_PHERM + DEMAND_WEIGHT_COEFF * demand[j][1]
        n_pherm[i][demand[j][0]] += tmp
        max_phm = max(max_phm, n_pherm[i][demand[j][0]])
    # -------------------------------
    # Scaling demand phermone to be
    # less than 1, ensures phermone
    # function behavior and limits
    # maximum advantage possible in
    # the entire system.
    # -------------------------------
    lower_lim = 10**(-5)
    if max_phm > 1:
      scl = 10**(-int(log10(max_phm)) - 1)
      for i in range(len(demand)):
        for j in range(len(nodes)):
          n_pherm[i][j] *= scl
          while n_pherm[i][j] < lower_lim:
            n_pherm[i][j] *= 10
    # -------------------------------
    print("Verified connections and set phermones!")
    self.sp_poss = sp_poss
    self.n_pherm = n_pherm
    self.llep_d = llep_d
    self.lep_t = lep_t
    self.let_t = let_t
    for i in range(len(demand)):
      if len(sp_poss[i][1]) - len(llep_d[i][1]) > 0:
        print("BAD LOCAL TO DEMAND PATH CONSTRUCTION: ", set(sp_poss[i][1]), set(len(llep_d[i][1])))
        exit(1)
      if len(sp_poss[i][0]) - len(llep_d[i][0]) > 0:
        print("BAD LOCAL FROM DEMAND PATH CONSTRUCTION: ", set(sp_poss[i][1]), set(len(llep_d[i][1])))
        exit(1)
    # for i in range(len(nodes)):
    #   for j in range(len(nodes)):
    #     if lep_t[0][j][0] != -1:
    #       cur, cur_id = lep_t[0][j]
    #       tpr = str(j)
    #       while cur != -1:
    #         tpr += " -> " + str(cur)
    #         cur, cur_id = lep_t[0][cur]
    #       print(tpr)
    #   break
    for i in range(len(nodes)):
      for j in range(len(nodes)):
        if let_t[i][j] == float('inf'):
          if lep_t[i][j][0] != -1:
            print("Error: these nodes have malformed path data:", i, j, lep_t[i][j])
    # print(llep_d[0])
    # print(sp_poss[0])
    # lep_frm, lep_to = llep_d[0]
    # prv = list(lep_to.keys())[0]
    # cur, cur_ty = lep_to[prv]
    # ptr = "" + str(prv)
    # while cur != -1:
    #   if cur_ty:
    #     e = edges[prv][cur]
    #   else:
    #     e = dedges[prv][cur]
    #   # print(prv, cur, e[0])
    #   prv = e[0]
    #   cur, cur_ty = lep_to[e[0]]
    #   ptr += " -> " + str(prv)
    # print(ptr)
    # cur, cur_id, cur_ty = lep_frm[list(lep_frm.keys())[0]]
    # ptr = str(cur)
    # while cur != -1:
    #   if cur_ty:
    #     e = edges[cur][cur_id]
    #   else:
    #     e = dedges[cur][cur_id]
    #   cur, cur_id, cur_ty = lep_frm[cur]
    #   ptr += " -> " + str(cur)
    # print(ptr)
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
    STAGNANT_LIMIT, STATUS_BREAK = int(0.2 * K), 1
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
    SJI_PHERM_SIZE = (DEMAND_SIZE + 1) * (DEMAND_SIZE + 1)
    SJI_PHERM_INIT, DELTA_SP_COEFF, DELTA_SP_NBH_COEFF = 0.55, 1.15, 0.75
    n_pherm = mp.Array('f', N_PHERM_SIZE, lock=False)
    sji_pherm = mp.Array('f', SJI_PHERM_SIZE, lock=False)
    c = -1
    for i in range(DEMAND_SIZE + 1):
      c = i * NUM_NODES
      for j in range(NUM_NODES):
        n_pherm[c + j] = self.n_pherm[i][j]
    for i in range(SJI_PHERM_SIZE):
      sji_pherm[i] = SJI_PHERM_INIT
    c = -1
    for i in range(DEMAND_SIZE + 1):
      self.sji_pherm.append([])
      c = i * (DEMAND_SIZE + 1)
      for j in range(DEMAND_SIZE + 1):
        self.sji_pherm[i].append(SJI_PHERM_INIT)
    cycles = [(mp.Value('f', lock=False),
               mp.Array('i', DEMAND_SIZE, lock=False),
               mp.Array('i', SWP_SIZE, lock=False)) for _ in range(ants_per_iter)]
    let_t = self.let_t      # not changing
    sp_poss = self.sp_poss  # not changing
    llep_d = self.llep_d    # not changing
    lep_t = self.lep_t      # not changing
    edges = self.edges      # not changing
    dedges = self.dedges    # not changing
    # print("Setting up delta hyperparamter...")
    # sample_eng = _aco_worker(barrier, saw_zero, demand, sp_poss, n_pherm, sji_pherm,
    #                          cycles[i][1], llep_d, lep_t, cycles[i][2], let_t, 1, 
    #                          DRONE_GROUND_SPEED, edges, dedges, cycles[i][0])
    # print("Set up delta hyperparamter!")
    processes = [mp.Process(target=_aco_worker,
                            args=(barrier, saw_zero, demand, sp_poss, n_pherm, sji_pherm,
                                  cycles[i][1], llep_d, lep_t, cycles[i][2], let_t, K, 
                                  DRONE_GROUND_SPEED, edges, dedges, cycles[i][0])) for i in range(ants_per_iter)]
    print("Initialized ACO child workers!\nStarting ACO...")
    best_cycle = sample([i for i in range(DEMAND_SIZE)], k=DEMAND_SIZE)
    best_swp = [-3 for _ in range(SWP_SIZE)]  # -3 ensures no initial activation
    best_energy = float(10**50)  # large enough number to compare against for us
    best_ind, j, delta, delta_sp, delta_sp_nbh, cyc_ind = -1, -1, -1, -1, -1, -1
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
            barrier.value += ants_per_iter
          print("Limit for iterations to stay stagnant exceeded! Stopping earlier by", K - iter,"iterations")
          break
      # elitism loading below
      delta = q / best_energy
      delta_sp = DELTA_SP_COEFF * delta
      delta_sp_nbh = DELTA_SP_NBH_COEFF * delta_sp
      first_parent, last_drone_del = None, None
      cycle, swp = best_cycle, best_swp
      j, cyc_ind, prev_tsj = 1, 0, False
      n_pherm[N_PHERM_LAST + demand[cycle[0]][0]] += delta
      if swp[0] >= 0:
        n_pherm[N_PHERM_LAST + swp[0]] += delta_sp
        for e in edges[swp[0]]:
          n_pherm[N_PHERM_LAST + e[0]] += delta_sp_nbh
        first_parent = cycle[0]
        last_drone_del = first_parent
      while j < SWP_SIZE - 1:
        if swp[j] == -2:
          if swp[j+1] == -2:
            if prev_tsj:
              n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
            else:
              sji_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
              prev_tsj = True
          elif swp[j+1] == -1:
            n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
          else:
            last_drone_del = cycle[cyc_ind + 1]
            n_pherm[demand[last_drone_del][0] + NUM_NODES * cycle[cyc_ind]] += delta
        elif swp[j] >= 0:
          n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * last_drone_del] += delta
          if prev_tsj:
            n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
          else:
            sji_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
          if swp[j+1] >= 0:
            n_pherm[swp[j+1] + NUM_NODES * last_drone_del] += delta_sp
            for e in edges[swp[j+1]]:
              n_pherm[e[0] + shft] += delta_sp_nbh
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
            n_pherm[e[0] + shft] += delta_sp_nbh
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
        delta_sp_nbh = DELTA_SP_NBH_COEFF * delta_sp
        first_parent, last_drone_del = None, None
        j, cyc_ind, prev_tsj = 1, 0, False
        n_pherm[N_PHERM_LAST + demand[cycle[0]][0]] += delta
        if swp[0] >= 0:
          n_pherm[N_PHERM_LAST + swp[0]] += delta_sp
          for e in edges[swp[0]]:
            n_pherm[N_PHERM_LAST + e[0]] += delta_sp_nbh
          first_parent = cycle[0]
          last_drone_del = first_parent
        while j < SWP_SIZE - 1:
          if swp[j] == -2:
            if swp[j+1] == -2:
              if prev_tsj:
                n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
              else:
                sji_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
                prev_tsj = True
            elif swp[j+1] == -1:
              n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
            else:
              last_drone_del = cycle[cyc_ind + 1]
              n_pherm[demand[last_drone_del][0] + NUM_NODES * cycle[cyc_ind]] += delta
          elif swp[j] >= 0:
            n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * last_drone_del] += delta
            if prev_tsj:
              n_pherm[demand[cycle[cyc_ind + 1]][0] + NUM_NODES * cycle[cyc_ind]] += delta
            else:
              sji_pherm[cycle[cyc_ind + 1] + DEMAND_SIZE * first_parent] += delta
            if swp[j+1] >= 0:
              n_pherm[swp[j+1] + NUM_NODES * last_drone_del] += delta_sp
              for e in edges[swp[j+1]]:
                n_pherm[e[0] + shft] += delta_sp_nbh
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
              n_pherm[e[0] + shft] += delta_sp_nbh
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
      if iter % STATUS_BREAK == 0:
        print("\nUpdate: best energy cycle found so far:", round(best_energy / 10**6, 2), "MJ")
      if iter < 0.1 * K:
        # Dynamic initial delta loading.
        q = 0.01 * best_energy
      for p in processes:
        if not p.is_alive():
          print("NOTE: Ant", p.pid, "got killed.")
          p.join()
          p.close()
          ants_per_iter -= 1
      c = 1
      while c > 0:
        with saw_zero.get_lock():
          c = saw_zero.value
      with barrier.get_lock():
        barrier.value += ants_per_iter
    pbar.close()
    print("ACO complete!")
    for p in processes:
      p.join()
      p.close()
    for i in range(DEMAND_SIZE + 1):
      c = i * NUM_NODES
      for j in range(NUM_NODES):
        self.n_pherm[i][j] = n_pherm[c + j]
    for i in range(DEMAND_SIZE + 1):
      c = i * (DEMAND_SIZE + 1)
      for j in range(DEMAND_SIZE + 1):
        self.sji_pherm[i][j] = sji_pherm[c + j]
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

  def make_phermone_plot(self, dem_ind=0, width=500, height=500, filename="phermone plot.png"):
    max_x, max_y = 0, 0
    for i in range(len(self.nodes)):
      if abs(self.nodes[i][0]) > max_x:
        max_x = self.nodes[i][0]
      if abs(self.nodes[i][1]) > max_y:
        max_y = self.nodes[i][1]
    w_sc, h_sc = 0.5 * width / max_x, 0.5 * height / max_y
    src = self.n_pherm[dem_ind]
    img = Image.new( 'RGB', (width+5, height+5), "black")
    pixels = img.load()
    for i in range(len(self.nodes)):
      R, G, B = pixels[int((self.nodes[i][0] + max_x) * w_sc), 
                       int((self.nodes[i][1] + max_y) * h_sc)]
      comp = int(10 + 10 * src[i])
      R = min(R + comp, 250)
      G = min(G + (250 - comp), 250)
      pixels[int((self.nodes[i][0] + max_x) * w_sc), 
             int((self.nodes[i][1] + max_y) * h_sc)] = (R, G, B)
    img.show()
    # img.save(filename)

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
