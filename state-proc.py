import pickle
from math import sqrt, sin, cos, exp, pi, atan2, tan
import multiprocessing as mp
from os import cpu_count
from tqdm import tqdm

GET_MONTH_INDEX = {"January":0,
                   "February":1,
                   "March":2,
                   "April":3,
                   "May":4,
                   "June":5,
                   "July":6,
                   "August":7,
                   "September":8,
                   "October":9,
                   "November":10,
                   "December":11}

def RH(isMorning, month):
    if isMorning:
        if month == 0:
            return 0.66
        elif month == 1:
            return 0.65
        elif month == 2:
            return 0.64
        elif month == 3:
            return 0.64
        elif month == 4:
            return 0.73
        elif month == 5:
            return 0.76
        elif month == 6:
            return 0.75
        elif month == 7:
            return 0.77
        elif month == 8:
            return 0.78
        elif month == 9:
            return 0.74
        elif month == 10:
            return 0.71
        else:
            return 0.69
    else:
        if month == 0:
            return 0.55
        elif month == 1:
            return 0.53
        elif month == 2:
            return 0.50
        elif month == 3:
            return 0.45
        elif month == 4:
            return 0.52
        elif month == 5:
            return 0.55
        elif month == 6:
            return 0.53
        elif month == 7:
            return 0.54
        elif month == 8:
            return 0.56
        elif month == 9:
            return 0.55
        elif month == 10:
            return 0.57
        else:
            return 0.59

D_PRES = 7
isMorning = False
Month = "May"
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
DRONE_SPAN = 1.58
ENG_ZERO = 10**(-50)

def kgs_to_W(kgs):
  return kgs * kgs_coeff

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
  DRONE_WEIGHT = kgs_to_W(13)

def copy_globals_energy(drone_speed):
  global C_D_ALPHA0, S_REF, DRONE_GROUND_SPEED, \
         CHORD, BETA, SINPSI, COSPSI, DRONE_WEIGHT
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
  DRONE_WEIGHT = kgs_to_W(13)

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

def get_decomposed_network(obj):
    xl, yl, uidl = obj.xl, obj.yl, obj.uidl
    ulb, vlb, lenlb = obj.ulb, obj.vlb, obj.lenlb
    uld, vld, lenld = obj.uld, obj.vld, obj.lenld
    avg_x = round(sum(x for x in xl) / len(xl), D_PRES)
    avg_y = round(sum(y for y in yl) / len(yl), D_PRES)
    for i in range(len(xl)):
        xl[i] = round((xl[i] - avg_x), max(D_PRES - 3, 0))
    for i in range(len(yl)):
        yl[i] = round((yl[i] - avg_y), max(D_PRES - 3, 0))
    min_x, min_y = min(x for x in xl), min(y for y in yl)
    max_x, max_y = max(x for x in xl), max(y for y in yl)
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
    fill_edge_data(dedges, DEDGE_WORK)
    print("Built internal edges structure & calibrated winds!")
    return (nodesl, edgesl, dedges, UID_to_ind, ind_to_UID)

def copy_decomposed_network(obj, src_dedges):
    xl, yl, uidl = obj.xl, obj.yl, obj.uidl
    ulb, vlb, lenlb = obj.ulb, obj.vlb, obj.lenlb
    uld, vld, lenld = obj.uld, obj.vld, obj.lenld
    avg_x = round(sum(x for x in xl) / len(xl), D_PRES)
    avg_y = round(sum(y for y in yl) / len(yl), D_PRES)
    for i in range(len(xl)):
        xl[i] = round((xl[i] - avg_x), max(D_PRES - 3, 0))
    for i in range(len(yl)):
        yl[i] = round((yl[i] - avg_y), max(D_PRES - 3, 0))
    min_x, min_y = min(x for x in xl), min(y for y in yl)
    max_x, max_y = max(x for x in xl), max(y for y in yl)
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
    x, y, mul_y, mul_x = 0, 0, 0, 0
    truck_speed, truck_epm = 0, 0
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
    search_list = [[e[0] for e in lst] for lst in src_dedges]
    for j in range(len(ulb)):
        if not ((ulb[j] in UID_to_ind) and (vlb[j] in UID_to_ind)):
            continue
        u_ind = UID_to_ind[ulb[j]]
        v_ind = UID_to_ind[vlb[j]]
        if u_ind == v_ind:    # removing cyclic edges.
            continue
        length = round(lenlb[j], max(D_PRES - 3, 0))
        dedges[u_ind].append((v_ind, length, src_dedges[u_ind][search_list[u_ind].index(v_ind)][2]))
    print("Built internal edges structure & calibrated winds!")
    return (nodesl, edgesl, dedges, UID_to_ind, ind_to_UID)

def power(rho, W, V_w_hd, V_w_lt):
  V = DRONE_GROUND_SPEED
  W *= kgs_coeff
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
  lCOEFF = rho * 6.28318 * 0.0005
  dCOEFF = rho * 0.00999 * 0.0005
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
      TBET, HBET, QBET = abs(resT) * 0.01, resH * 0.01, resQ * 0.01
      omegaN = sqrt(T0/TBET) * omega0
      # print("QBET:",QBET,"TBET:",TBET,"HBET:",HBET,"O0:",omega0,"ON:",omegaN)
      omega0 = omegaN
    T0 = TBET
    H0 = HBET
  # print((W/6)-T0) # excess
  # omega_to_RPM(omegaN), rad_to_deg(alpha_D)
  # return QBET * omegaN
  # print(TBET*101.97, "g,", QBET, "Nm", omega_to_RPM(omegaN),"RPM")
  return (omegaN * QBET * 6 * 1.0145 / 0.77)
  # assumes each of the 6 motors has 77% efficiency.

def fill_edge_data(dedges, dedge_work):
  print("Logical number of CPUs:", cpu_count())
  p = mp.Pool(processes=cpu_count(), 
              initializer=copy_globals_energy,
              initargs=(DRONE_GROUND_SPEED,))
  pbar = tqdm(total=len(dedge_work))
  def update(*a):
    pbar.update()
  u_ind, v_ind, length = 0, 0, 0
  rho, V_w_hd, V_w_lt = 0, 0, 0
  async_obj, ind = 0, 0
  for i in range(len(dedge_work)):
    u_ind, ind, rho, V_w_hd, V_w_lt = dedge_work[i]
    dedge_work[i] = (u_ind, ind, p.apply_async(_energy_worker, (rho, V_w_hd, V_w_lt), callback=update))
  for i in range(len(dedge_work)):
    u_ind, ind, async_obj = dedge_work[i]
    v_ind, length = dedges[u_ind][ind]
    dedges[u_ind][ind] = (v_ind, length, async_obj.get())
  p.close()
  p.join()
  pbar.close()

def _energy_worker(rho, V_w_hd, V_w_lt):
  # return WEIGHTS
  drone_power = []
  for w in WEIGHTS:
    drone_power.append(power(rho, w, V_w_hd, V_w_lt))
  return tuple(drone_power)

class EnergyHelper:
  """
  stores nodes and edges globally.
  """
  def __init__(self, nodes: list, edges: list, dedges: list, UID_to_ind, 
               ind_to_UID, angle_tolerance, demand=[]):
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
    input = open(filename, 'rb')
    obj = pickle.load(input)
    input.close()
    ehobj = EnergyHelper(obj.nodes,
                         obj.edges,
                         obj.dedges,
                         obj.UID_to_ind,
                         obj.ind_to_UID,
                         obj.ang_tol,
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

def count_non_empty_lists(lst):
   cnt = 0
   for i in lst:
      if len(i) > 0:
         cnt += 1
   return cnt

if __name__ == '__main__':
#   eh = el.EnergyHelper.load("uoft.pkl")
#   eh.save("manhattan-pre.pkl")
  TRUCK_PROFILES = [(12, 1.4), (15, 1.6), (20, 1.8)]

  for V in range(5, 26, 5):
    init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
                 base_temperature=14, temp_flucts_coeff=3, drone_speed=V,
                 relative_humidity=RH(isMorning,GET_MONTH_INDEX[Month]))

    SET_STRING = "pickles/set-{}.pkl".format(1)
    nodes, edges, dedges, UID_to_ind, ind_to_UID = get_decomposed_network(Storage.load(SET_STRING))
    eh = EnergyHelper(nodes, edges, dedges, UID_to_ind, ind_to_UID, 10**(-2))
    eh.save("manhattan-policy-set-{}-{}ms.pkl".format(1, V))
  # for V in range(5, 26, 5):
  #   init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
  #                base_temperature=14, temp_flucts_coeff=3, drone_speed=V,
  #                relative_humidity=RH(isMorning,GET_MONTH_INDEX[Month]))
  
  #   SET_STRING = "set-{}.pkl".format(2)
  #   nodes, edges, dedges, UID_to_ind, ind_to_UID = get_decomposed_network(Storage.load(SET_STRING))
  #   eh = EnergyHelper(nodes, edges, dedges, UID_to_ind, ind_to_UID, 10**(-2))
  #   eh.save("manhattan-policy-set-{}-{}ms.pkl".format(2, V))
  # for V in range(5, 26, 5):
  #   init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
  #                base_temperature=14, temp_flucts_coeff=3, drone_speed=V,
  #                relative_humidity=RH(isMorning,GET_MONTH_INDEX[Month]))
  
  #   SET_STRING = "set-{}.pkl".format(3)
  #   src_dedges = EnergyHelper.load("manhattan-policy-set-{}-{}ms.pkl".format(2, V)).dedges
  #   nodes, edges, dedges, UID_to_ind, ind_to_UID = copy_decomposed_network(Storage.load(SET_STRING), src_dedges)
  #   eh = EnergyHelper(nodes, edges, dedges, UID_to_ind, ind_to_UID, 10**(-2))
  #   eh.save("manhattan-policy-set-{}-{}ms.pkl".format(3, V))
  # for V in range(5, 26, 5):
  #   init_globals(max_truck_speed=12, base_truck_speed=1.4, truck_city_mpg=24,
  #                base_temperature=14, temp_flucts_coeff=3, drone_speed=V,
  #                relative_humidity=RH(isMorning,GET_MONTH_INDEX[Month]))

  #   SET_STRING = "set-{}.pkl".format(4)
  #   src_dedges = EnergyHelper.load("manhattan-policy-set-{}-{}ms.pkl".format(2, V)).dedges
  #   nodes, edges, dedges, UID_to_ind, ind_to_UID = copy_decomposed_network(Storage.load(SET_STRING), src_dedges)
  #   eh = EnergyHelper(nodes, edges, dedges, UID_to_ind, ind_to_UID, 10**(-2))
  #   eh.save("manhattan-policy-set-{}-{}ms.pkl".format(4, V))
  # for V in range(5, 26, 5):
  #   eh = EnergyHelper.load("pickles/manhattan-policy-set-{}-{}ms.pkl".format(1, V))
  #   print(1, V, count_non_empty_lists(eh.dedges), count_non_empty_lists(eh.edges), count_non_empty_lists(eh.nodes))
  # for i in range(2, 5):
  #    for V in range(5, 26, 5):
  #       eh = EnergyHelper.load("manhattan-policy-set-{}-{}ms.pkl".format(i, V))
  #       print(i, V, count_non_empty_lists(eh.dedges), count_non_empty_lists(eh.edges), count_non_empty_lists(eh.nodes))
  # exit(0)

