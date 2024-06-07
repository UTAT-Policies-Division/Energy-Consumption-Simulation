from math import sqrt, acos, atan2, pi, \
                 sin, cos, tan, exp, \
                 floor, ceil
import matplotlib.pyplot as plt


half_pi = pi / 2
three_half_pi = 3 * (pi / 2)
two_pi = 2 * pi
RPM_coeff = two_pi / 60
meter_coeff = 39.3700787
mps_coeff = 3.6
rho_air_std = 1.204 # kg / m^3
kgs_coeff = 9.81
deg_coeff = pi / 180
PROP_START = 1  # inches
PROP_END = 11   # inches
REF_ANG_DEC = 2
REF_ANG = 10**(-REF_ANG_DEC)
ANG_INT = [x*REF_ANG for x in range(0, ceil(two_pi / REF_ANG))]
ANG_INT_ITER = list(range(len(ANG_INT)))
CHORD_DATA = [-1 for _ in range(PROP_START*100, (PROP_END*100)+1)]
CHORD_DATA_ITER = list(range(len(CHORD_DATA)))
ANG_SIN_DATA = [sin(psi) for psi in ANG_INT]
ANG_COS_DATA = [cos(psi) for psi in ANG_INT]
DRAW_PREC = 100 # power of 10, larger => more precise
AREA = pi * (11 / meter_coeff)**2
NEWT_PREC = 10**(-5)

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

def c(r):   # O(1) via lookup table, correct, but prefer not used
  """
  chord lengths for 1 <= r <= 11 inches, 
  result in mm, for 22x8 propeller
  assumes safe use for efficiency.
  """
  return CHORD_DATA[int((r-1) * 100)]

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

def get_init_data():
  CHORD = []  # holds in millimeters
  BETA = []
  _r = 1.0
  d = 0.1
  tmp, tmp1 = 0, 0
  for _ in range(0, 100):
    r = _r
    if r <= 4:
      r -= 4
      tmp = r
      r *= r
      tmp1 = 0.652444*cos(3.14425*tmp) + 50.6977 - \
             1.20882*tmp + (1.58523 + 3.23691*tmp)*r
      r *= r
      CHORD.append(tmp1 - 0.208061*r*tmp)
    elif r <= 7:
      r -= 4
      tmp = r
      r *= r
      CHORD.append(0.114129*cos(2.41374*tmp) + 51.2251 - \
                   0.253086*tmp - (1.00919 - 0.0548433*tmp)*r)
    else:
      tmp = r
      r *= r
      CHORD.append(-1*exp(-143.87179093 + 13.3561*tmp) + \
                   63.8221 - (0.55019 - 0.0178557*tmp)*r)
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
  return (CHORD, BETA, SINPSI, COSPSI)

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

  def __init__(self, nodes, edges, angle_tolerance, gen_plot_data=False):
    self.nodes = nodes
    self.edges = edges
    self.ang_tol = angle_tolerance
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
  
  def D_f(self, rho, V):   # correct, but prefer not used
    """
    Drag force.
    TODO: include effect of having a package.
    """
    return 0.5 * self.C_D_alph0 * self.S_ref * rho * V * V

  def thrust(self, rho, V, W, V_w_hd, V_w_lt, CHORD, BETA, SINPSI, COSPSI):
    W += kgs_to_W(13)    # needs update, drone base weight
    Df = 0.5 * self.C_D_alph0 * self.S_ref * rho * V * V
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
    while abs(T0 - T_OLD) / T0 > 0.000001:
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
      while abs(fv) > NEWT_PREC:
        k_v -= dv
        VAR1 = k_v + Vc
        fv = ((k_v * k_v * (VAR1 * VAR1 + Vxsq)) - C)
        dv = fv / (2 * k_v * ((VAR1 * (VAR1 + k_v)) + Vxsq))
      # print("alpha_d:",rad_to_deg(alpha_D),"T0:",T0,"v0:",k_v,"Vx:",Vx,"Vc:",Vc)
      while abs(TBET - T0) / T0 > 0.000001:
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
