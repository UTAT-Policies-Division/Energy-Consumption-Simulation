from random import randint, sample, choices
import heapq

NEWT_PREC = 10**(-5)
BATTERY_RESERVE_MARGIN = 0.2
BATTERY_CAPACITY = 17.0 * 42 * 3600 / 30 # J,        TODO: remove divider
MAX_BATTERY_USE = BATTERY_CAPACITY * (1 - BATTERY_RESERVE_MARGIN)
MAX_BATTERY_USE_HALF = MAX_BATTERY_USE / 2
MIN_MEETUP_BATTERY_REM = MAX_BATTERY_USE * 0.15
TIME_TO_FULL_CHARGE = 3600
RECARGE_RATE = BATTERY_CAPACITY / TIME_TO_FULL_CHARGE
WEIGHTS = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2]
ENG_ZERO = 10**(-50)

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
      prv = e[0]
      cur, cur_ty = lep_to[prv]
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
    prv = e[0]
    cur, cur_ty = lep_to[prv]
  return time

def construct_time_truck(lep_t, from_node, to_node, edges):
  time = 0
  cur, cur_id = lep_t[from_node][to_node]
  if cur == -1:
    if from_node == to_node:
      return 0
    else:
      return float('inf')
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
      prv = e[0]
      cur, cur_ty = lep_to[prv]
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
    prv = e[0]
    cur, cur_ty = lep_to[prv]
  return eng

def construct_energy_launch(llep_d, to_dem, sp, edges, dedges, demand, dron_w, DS):
  eng, prv, cur, cur_ty, w_ind = ENG_ZERO, sp, -1, -1, int(dron_w * 4)
  _, lep_to = llep_d[to_dem]
  cur, cur_ty = lep_to[prv]
  if cur == -1:
    if sp == demand[to_dem][0]:
      return ENG_ZERO
    else:
      return float('inf')
  while cur != -1:
    if cur_ty:
      e = edges[prv][cur]
      eng += e[4][w_ind] * e[1] / DS
    else:
      e = dedges[prv][cur]
      eng += e[2][w_ind] * e[1] / DS
    prv = e[0]
    cur, cur_ty = lep_to[prv]
  return eng

def construct_energy_spef(llep_d, from_dem, sp, to_dem, edges, dedges, demand, dron_w, DS):
  eng, cur, cur_ty, cur_id, w_ind = ENG_ZERO, -1, -1, -1, int(dron_w * 4) 
  lep_frm, _ = llep_d[from_dem]
  _, lep_to = llep_d[to_dem]
  prv = sp
  cur, cur_ty = lep_to[prv]
  if cur == -1:
    if sp != demand[to_dem][0]:
      return float('inf')
  while cur != -1:
    if cur_ty:
      e = edges[prv][cur]
      eng += e[4][w_ind] * e[1] / DS
    else:
      e = dedges[prv][cur]
      eng += e[2][w_ind] * e[1] / DS
    prv = e[0]
    cur, cur_ty = lep_to[prv]
  cur, cur_id, cur_ty = lep_frm[sp]
  if cur == -1:
    if sp == demand[from_dem][0]:
      return eng
    else:
      return float('inf')
  while cur != -1:
    if cur_ty:
      e = edges[cur][cur_id]
      eng += e[4][w_ind] * e[1] / DS
    else:
      e = dedges[cur][cur_id]
      eng += e[2][w_ind] * e[1] / DS
    cur, cur_id, cur_ty = lep_frm[cur]
  return eng

def construct_energy_meetup(llep_d, from_dem, sp, edges, dedges, demand, DS):
  eng, cur, cur_ty, cur_id = ENG_ZERO, -1, -1, -1
  lep_frm, _ = llep_d[from_dem]
  cur, cur_id, cur_ty = lep_frm[sp]
  if cur == -1:
    if sp == demand[from_dem][0]:
      return ENG_ZERO
    else:
      return float('inf')
  while cur != -1:
    if cur_ty:
      e = edges[cur][cur_id]
      eng += e[4][0] * e[1] / DS
    else:
      e = dedges[cur][cur_id]
      eng += e[2][0] * e[1] / DS
    cur, cur_id, cur_ty = lep_frm[cur]
  return eng

def _aco_worker(barrier, saw_zero, demand, sp_poss, n_pherm, sji_pherm, cycle, 
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
  sp_poss_set, nbs, ws, seen, lmem, not_got_chance = [], [], [], [], [], False
  for i in range(len(sp_poss)):
    sp_poss_set.append((set(sp_poss[i][0]),set(sp_poss[i][1])))
  eng_rem, eng_to_add, eng_at_sp, next_dem_node, curr_shft, flag = 0, 0, 0, -1, -1, -1
  # -----------------------------
  # State Holders
  # drone_loc, truck_loc are indexes of demand array.
  # if truck waiting at a switch point, sel_sp is set.
  # -----------------------------
  truck_loc, truck_loc_node, truck_w, w_coeff, parent_loc_node, sp = None, None, None, None, None, None
  drone_loc, drone_w, to_visit, to_visit_truck, tvs_ind_st, parent_loc = None, None, [], [], None, None
  next_dem, eng_tot, num_delvd, swp_ind, eng_acc, pot_dem, e = None, None, None, None, None, None, None
  w_coeff_oth, truck_w_og, best_eng, best_sp, best_eng_to_add, sel_sp = None, None, None, None, None, None
  time_taken, prev_t_eng, truck_w_og, truck_side_eng, truck_side_w_lost = None, None, None, None, None
  pot_dem_node, lep_frm, lep_to, eng, prv, cur, cur_ty, cur_id, w_ind = None, None, None, -1, -1, -1, -1, -1, -1
  lmem = [-1 for _ in range(len(WEIGHTS))]
  _glb_cons_eng_meetup = [[float('inf') for _ in range(NUM_NODES)] for _ in range(DEMAND_SIZE)]
  for i in range(DEMAND_SIZE):
    for j in sp_poss[i][0]:
      if _glb_cons_eng_meetup[i][j] != float('inf'):
        continue
      eng = ENG_ZERO
      lep_frm = llep_d[i][0]
      cur, cur_id, cur_ty = lep_frm[j]
      if cur == -1:
        if j == demand[i][0]:
          _glb_cons_eng_meetup[i][j] = ENG_ZERO
        continue
      while cur != -1:
        if cur_ty:
          e = edges[cur][cur_id]
          eng += e[4][0] * e[1] / DS
        else:
          e = dedges[cur][cur_id]
          eng += e[2][0] * e[1] / DS
        cur, cur_id, cur_ty = lep_frm[cur]
      _glb_cons_eng_meetup[i][j] = eng
  # -----------------------------
  while K > 0:
    print("Entered Iteration")
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
    drone_loc, drone_w, ENG_LEVL = -1, 0, MAX_BATTERY_USE
    for j in range(DEMAND_SIZE):
      f_dem_ws[j] = (n_pherm[N_PHERM_LAST + demand[j][0]])**ALPHA / \
                    (let_t[src][demand[j][0]] / w_coeff)**BETA
      got[j] = 0
    next_dem = choices(f_dem_ps, weights=f_dem_ws)[0]
    got[next_dem] = 1
    cycle[num_delvd] = next_dem
    num_delvd += 1
    if demand[next_dem][1] <= MAX_WEIGHT:
      # sample a switch point, if delivery node itself then none.
      nbs, ws = [], []
      eng_rem = ENG_LEVL - MIN_MEETUP_BATTERY_REM
      for sp in sp_poss[next_dem][1]:
        eng_to_add = construct_energy_launch(llep_d, next_dem, sp, edges, dedges, demand, demand[next_dem][1], DS)
        if eng_rem - eng_to_add > 0:
          nbs.append((sp, eng_to_add))
          ws.append((n_pherm[N_PHERM_LAST + sp])**ALPHA / 
                    ((let_t[src][sp] / w_coeff) + eng_to_add)**BETA)
      if len(nbs) > 0:
        sel_sp, eng_to_add = choices(nbs, weights=ws)[0]
        if sel_sp == next_dem:
          sel_sp = -1
    if sel_sp == -1:
      # not going to be allocating a switch point.
      # cycle[num_delvd] = next_dem
      # num_delvd += 1
      truck_loc, truck_loc_node = next_dem, demand[next_dem][0]
      eng_tot += let_t[src][truck_loc_node] / w_coeff
      # print("initial deployment no switch point", truck_w)
      truck_w -= demand[next_dem][1]
      # print("Removed", demand[next_dem][1])
      swp[swp_ind] = -1
    else:
      # allocated a switch point.
      truck_loc, truck_loc_node, drone_loc = -1, -1, next_dem
      drone_w = demand[next_dem][1]
      # print("initial deployment with switch point", truck_w)
      truck_w -= DRONE_WEIGHT + drone_w
      # print("Removed", DRONE_WEIGHT + drone_w)
      # print("eng total before", eng_tot)
      eng_tot += (let_t[src][sel_sp] / w_coeff)
      # print("eng total after adding", (let_t[src][sel_sp] / w_coeff), ", is", eng_tot)
      to_visit.append(sel_sp)
      to_visit.append((next_dem, eng_to_add))
      swp[swp_ind] = sel_sp
    w_coeff = 1 - (truck_w / TW_DENM)
    swp_ind += 1
    # -----------------------------
    while num_delvd < DEMAND_SIZE or drone_loc >= 0:
      if drone_loc >= 0:
        # giving drone the option to continue delivering
        # length of to_visit, to_visit_truck must be 0.
        eng_acc, tvs_ind_st, truck_w_og = 0, 2, truck_w
        time_passed = construct_time(llep_d, 0, to_visit, edges, dedges, DS)
        nbs, ws = [], []
        prev_t_eng = 0
        parent_loc = to_visit[1][0]
        parent_loc_node = to_visit[0]
        # print("Entering drone doing deliveries section.")
        # jk = 0
        # for i in got:
        #   if i == 1:
        #     jk += 1
        # print("PRE num got set:", jk, "but num delvd", num_delvd)
        # print(to_visit)
        # print(to_visit_truck)
        for i in range(DEMAND_SIZE):
          if got[i] == 0:
            time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
            if time_passed - time_taken > 0 and let_t[demand[i][0]][drone_loc] != float('inf'):
              nbs.append((i, time_taken))
              ws.append((sji_pherm[i + DEMAND_SIZE * parent_loc])**ALPHA /
                        (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
        while len(nbs) > 0:
          # can move away from switch point for work!
          # print(parent_loc_node, nbs, ws)
          pot_dem, time_taken = choices(nbs, weights=ws)[0]
          # print("Found time from first step of a switch for truck to deliver", truck_w)
          truck_w -= demand[pot_dem][1]
          # print("Removed", demand[pot_dem][1])
          w_coeff = 1 - (truck_w / TW_DENM)
          to_visit_truck.append((pot_dem, 1, prev_t_eng + let_t[parent_loc_node][demand[pot_dem][0]]))
          # print("set for first drone")
          got[pot_dem] = 1
          time_passed -= time_taken
          parent_loc, _, prev_t_eng = to_visit_truck[-1]
          parent_loc_node = demand[parent_loc][0]
          nbs, ws = [], []
          for i in range(DEMAND_SIZE):
            if got[i] == 0:
              time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
              if time_passed - time_taken > 0 and let_t[demand[i][0]][drone_loc] != float('inf'):
                nbs.append((i, time_taken))
                ws.append((n_pherm[demand[i][0] + DEMAND_SIZE * parent_loc])**ALPHA /
                          (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
        cur = drone_w + 0.25
        cur_id = int(4 * cur)
        while cur <= MAX_WEIGHT:
          lmem[cur_id] = construct_energy(llep_d, to_visit, edges, dedges, cur, DS)
          cur += 0.25
          cur_id += 1
        while ENG_LEVL - eng_acc > 0:
          nbs, ws = [], []
          curr_shft = NUM_NODES * drone_loc
          for i in range(DEMAND_SIZE):
            if got[i] == 0 and \
               drone_w + demand[i][1] <= MAX_WEIGHT and \
               let_t[parent_loc_node][demand[i][0]] != float('inf'):
              common_sps = sp_poss_set[drone_loc][0] & sp_poss_set[i][1]
              if len(common_sps) > 0:
                eng_so_far = lmem[int((drone_w + demand[i][1]) * 4)]
                eng_rem = ENG_LEVL - eng_so_far - MIN_MEETUP_BATTERY_REM
                if eng_rem <= 0:
                  continue
                best_sp = -1
                for ind in common_sps:
                  # from_dem = drone_loc, sp = ind, to_dem = i
                  eng_to_add, w_ind = ENG_ZERO, int(demand[i][1] * 4)
                  lep_frm = llep_d[drone_loc][0]
                  lep_to = llep_d[i][1]
                  prv = ind
                  cur, cur_ty = lep_to[prv]
                  if cur == -1:
                    if ind != demand[i][0]:
                      continue
                  while cur != -1:
                    if cur_ty:
                      e = edges[prv][cur]
                      eng_to_add += e[4][w_ind] * e[1] / DS
                    else:
                      e = dedges[prv][cur]
                      eng_to_add += e[2][w_ind] * e[1] / DS
                    prv = e[0]
                    cur, cur_ty = lep_to[prv]
                  cur, cur_id, cur_ty = lep_frm[ind]
                  if cur == -1:
                    if ind != demand[drone_loc][0]:
                      continue
                  while cur != -1:
                    if cur_ty:
                      e = edges[cur][cur_id]
                      eng_to_add += e[4][w_ind] * e[1] / DS
                    else:
                      e = dedges[cur][cur_id]
                      eng_to_add += e[2][w_ind] * e[1] / DS
                    cur, cur_id, cur_ty = lep_frm[cur]
                  if eng_to_add < eng_rem:
                    eng_rem = eng_to_add
                    best_sp = ind
                if best_sp >= 0:
                  nbs.append((i, best_sp, eng_so_far + eng_rem))
                  ws.append((n_pherm[demand[i][0] + curr_shft])**ALPHA / (eng_so_far)**BETA)
          if len(nbs) == 0:
            break
          drone_loc, sp, eng_acc = choices(nbs, weights=ws)[0]
          got[drone_loc] = 1
          drone_w += demand[drone_loc][1]
          # print("Found a new drone location for drone to deliver to", truck_w)
          truck_w -= demand[drone_loc][1]
          # print("Removed", demand[drone_loc][1], "to get", truck_w)
          w_coeff = 1 - (truck_w / TW_DENM)
          to_visit.append(sp)
          to_visit.append((drone_loc, eng_acc))
          time_passed += construct_time(llep_d, tvs_ind_st, to_visit, edges, dedges, DS)
          nbs, ws = [], []
          if len(to_visit_truck) == 0:
            # prev_t_eng = 0
            # parent_loc = to_visit[1][0]
            # parent_loc_node = to_visit[0]
            for i in range(DEMAND_SIZE):
              if got[i] == 0:
                time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
                if time_passed - time_taken > 0 and let_t[demand[i][0]][drone_loc] != float('inf'):
                  nbs.append((i, time_taken))
                  ws.append((sji_pherm[i + DEMAND_SIZE * parent_loc])**ALPHA /
                            (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
          else:
            # parent_loc, _, prev_t_eng = to_visit_truck[-1]
            # parent_loc_node = demand[parent_loc][0]
            for i in range(DEMAND_SIZE):
              if got[i] == 0:
                time_taken = construct_time_truck(lep_t, parent_loc_node, demand[i][0], edges)
                if time_passed - time_taken > 0 and let_t[demand[i][0]][drone_loc] != float('inf'):
                  nbs.append((i, time_taken))
                  ws.append((n_pherm[demand[i][0] + DEMAND_SIZE * parent_loc])**ALPHA /
                            (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
          while len(nbs) > 0:
            # can move away from switch point for work!
            pot_dem, time_taken = choices(nbs, weights=ws)[0]
            # print("Found time during drone itenary for truck to deliver", truck_w)
            truck_w -= demand[pot_dem][1]
            # print("Removed", demand[pot_dem][1])
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
                if time_passed - time_taken > 0 and let_t[demand[i][0]][drone_loc] != float('inf'):
                  nbs.append((i, time_taken))
                  ws.append((n_pherm[demand[i][0] + DEMAND_SIZE * parent_loc])**ALPHA /
                            (let_t[parent_loc_node][demand[i][0]] / w_coeff)**BETA)
          tvs_ind_st = len(to_visit)
        nbs, ws = [], []
        # jk = 0
        # for i in got:
        #   if i == 1:
        #     jk += 1
        # print("POST num got set:", jk, "but num delvd", num_delvd)
        # print(to_visit)
        # print(to_visit_truck)
        # print("OG", truck_w)
        truck_w = truck_w_og
        tvs_ind_st, truck_side_eng, truck_side_w_lost = 0, 0, -1
        parent_loc, parent_loc_node = to_visit[1][0], demand[to_visit[1][0]][0]
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
          eng_rem = ENG_LEVL - to_visit[i][1]
          # print("Inside loop to select a demand", truck_w)
          truck_w -= truck_side_w_lost + demand[drone_loc][1]
          # print("Removed", truck_side_w_lost + demand[drone_loc][1])
          pot_dem = to_visit[i + 2][0]
          pot_dem_node = demand[pot_dem][0]
          w_coeff = 1 - (truck_w / TW_DENM)
          w_coeff_oth = 1 - ((truck_w + DRONE_WEIGHT) / TW_DENM)
          # to meetup with drone, find optimal
          # switch point to minimize total
          # consumption, phermone free !
          best_eng, best_sp = float('inf'), -1
          for ind in sp_poss[drone_loc][0]:
            tmp = let_t[parent_loc_node][ind] + let_t[ind][pot_dem_node]
            if tmp != float('inf'):
              eng_to_add = _glb_cons_eng_meetup[drone_loc][ind]
              if eng_rem - eng_to_add > 0: 
                tmp = eng_to_add + (let_t[parent_loc_node][ind] / w_coeff) + (let_t[ind][pot_dem_node] / w_coeff_oth)
                if tmp < best_eng:
                  best_eng = tmp
                  best_sp = ind
                  best_eng_to_add = eng_to_add
          if best_eng == float('inf'):
            # best_sp = drone_loc
            # best_eng_to_add = ENG_ZERO
            assert False, "Should always have a solution"
          nbs.append((i + 2, -2, best_sp, best_eng_to_add))
          if tvs_ind_st > 0:
            ws.append(((n_pherm[pot_dem_node + NUM_NODES * parent_loc] + 
                        (truck_w_og - truck_w) * PCKG_BONUS_COEFF)**ALPHA / 
                       ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                        (let_t[best_sp][pot_dem_node] / w_coeff_oth) +
                        best_eng_to_add + truck_side_eng)**BETA))
          else:
            ws.append(((sji_pherm[pot_dem + DEMAND_SIZE * parent_loc] + 
                        (truck_w_og - truck_w) * PCKG_BONUS_COEFF)**ALPHA / 
                       ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                        (let_t[best_sp][pot_dem_node] / w_coeff_oth) +
                        best_eng_to_add + truck_side_eng)**BETA))
        # all side jobs for last drone delivery
        truck_side_w_lost = 0
        while tvs_ind_st < len(to_visit_truck):
            truck_side_eng = to_visit_truck[tvs_ind_st][2]
            parent_loc = to_visit_truck[tvs_ind_st][0]
            parent_loc_node = demand[parent_loc][0]
            truck_side_w_lost += demand[parent_loc][1]
            tvs_ind_st += 1
        # print("Had more possible side deliveries to consider after lost drone delivery", truck_w)
        truck_w -= truck_side_w_lost
        # print("Removed", truck_side_w_lost)
        drone_loc = to_visit[-1][0]
        eng_rem = ENG_LEVL - to_visit[-1][1]
        w_coeff = 1 - (truck_w / TW_DENM)
        w_coeff_oth = 1 - ((truck_w + DRONE_WEIGHT) / TW_DENM)
        # print(to_visit, to_visit_truck)
        if (len(to_visit) / 2) + len(to_visit_truck) + num_delvd > DEMAND_SIZE: # if none remaining, allow src to be last
          pot_dem_node = src
          best_eng, best_sp = float('inf'), -1
          for ind in sp_poss[drone_loc][0]:
            tmp = let_t[parent_loc_node][ind] + let_t[ind][src]
            if tmp != float('inf'):
              eng_to_add = _glb_cons_eng_meetup[drone_loc][ind]
              if eng_rem - eng_to_add > 0:
                tmp = eng_to_add + (let_t[parent_loc_node][ind] / w_coeff) + (let_t[ind][src] / w_coeff_oth)
                if tmp < best_eng:
                  best_eng = tmp
                  best_sp = ind
                  best_eng_to_add = eng_to_add
          if best_eng == float('inf'):   # drone can't move at all.
            # best_sp = drone_loc
            # best_eng_to_add = ENG_ZERO
            assert False, "Should always have a solution"
          nbs.append((len(to_visit) + 1, -1, best_sp, best_eng_to_add))
          if tvs_ind_st > 0:
            ws.append(((n_pherm[src + NUM_NODES * parent_loc] + 
                        (truck_w_og - truck_w) * PCKG_BONUS_COEFF +
                        n_pherm[src + NUM_NODES * drone_loc])**ALPHA / 
                       ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                        (let_t[best_sp][src] / w_coeff_oth) +
                        best_eng_to_add + truck_side_eng)**BETA))
          else:
            ws.append(((sji_pherm[DEMAND_SIZE + DEMAND_SIZE * parent_loc] + 
                        (truck_w_og - truck_w) * PCKG_BONUS_COEFF +
                        n_pherm[src + NUM_NODES * drone_loc])**ALPHA / 
                       ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                        (let_t[best_sp][src] / w_coeff_oth) +
                        best_eng_to_add + truck_side_eng)**BETA))
        else:   # otherwise choose a new demand to consider post drone operation.
          for i in range(DEMAND_SIZE):
            if got[i] == 0:
              best_eng, best_sp = float('inf'), -1
              for ind in sp_poss[drone_loc][0]:
                tmp = let_t[parent_loc_node][ind] + let_t[ind][demand[i][0]]
                if tmp != float('inf'):
                  eng_to_add = _glb_cons_eng_meetup[drone_loc][ind]
                  if eng_rem - eng_to_add > 0:
                    tmp = eng_to_add + (let_t[parent_loc_node][ind] / w_coeff) + (let_t[ind][demand[i][0]] / w_coeff_oth)
                    if tmp < best_eng:
                      best_eng = tmp
                      best_sp = ind
                      best_eng_to_add = eng_to_add
              if best_eng == float('inf'):   # drone can't move at all.
                # best_sp = drone_loc
                # best_eng_to_add = ENG_ZERO
                assert False, "Should always have a solution"
              nbs.append((len(to_visit) + 1, i, best_sp, best_eng_to_add))
              if tvs_ind_st > 0:
                ws.append(((n_pherm[demand[i][0] + NUM_NODES * parent_loc] + 
                          (truck_w_og - truck_w) * PCKG_BONUS_COEFF +
                          n_pherm[demand[i][0] + NUM_NODES * drone_loc])**ALPHA / 
                         ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                          (let_t[best_sp][demand[i][0]] / w_coeff_oth) +
                          best_eng_to_add + truck_side_eng)**BETA))
              else:
                ws.append(((sji_pherm[i + DEMAND_SIZE * parent_loc] + 
                          (truck_w_og - truck_w) * PCKG_BONUS_COEFF +
                          n_pherm[demand[i][0] + NUM_NODES * drone_loc])**ALPHA / 
                         ((let_t[parent_loc_node][best_sp] / w_coeff) + 
                          (let_t[best_sp][demand[i][0]] / w_coeff_oth) +
                          best_eng_to_add + truck_side_eng)**BETA))
        # print("DRONE LOOP: ",len(nbs), num_delvd)
        # print(nbs, ws)
        ind, flag, sp, best_eng_to_add = choices(nbs, weights=ws)[0]
        # print("flag", flag)
        # print("Getting back original weight", truck_w_og, "+ drone weight")
        truck_w = truck_w_og + DRONE_WEIGHT
        # print("New weight", truck_w, "selected index", ind)
        parent_loc, parent_loc_node = to_visit[1][0], demand[to_visit[1][0]][0]
        # adding selected demand after end point confirmed
        # first drone itenary
        # drone_loc = to_visit[1][0]
        # cycle[num_delvd] = drone_loc
        # num_delvd += 1
        for i in range(3, ind, 2):
          drone_loc = to_visit[i][0]
          swp[swp_ind] = -2
          swp[swp_ind + 1] = to_visit[i - 1]
          swp_ind += 2
          cycle[num_delvd] = drone_loc
          num_delvd += 1
          # print("Delivering weights by drone as per itenary", truck_w)
          truck_w -= demand[drone_loc][1]
          # print("Removed", demand[drone_loc][1])
        # then truck itenary
        tvs_ind_st = 0
        # print(to_visit, to_visit_truck)
        for i in range(1, ind, 2):
          while tvs_ind_st < len(to_visit_truck) and to_visit_truck[tvs_ind_st][1] <= i:
            swp[swp_ind] = -2
            swp[swp_ind + 1] = -2
            swp_ind += 2
            parent_loc = to_visit_truck[tvs_ind_st][0]
            parent_loc_node = demand[parent_loc][0]
            cycle[num_delvd] = parent_loc
            num_delvd += 1
            # print("Removing itenaries done on the side", truck_w)
            truck_w -= demand[parent_loc][1]
            # print("Removed", demand[parent_loc][1])
            tvs_ind_st += 1
        # jk = 0
        # for i in got:
        #   if i == 1:
        #     jk += 1
        # print("PRE unset num got set:", jk, "but num delvd", num_delvd)
        # print(to_visit_truck, tvs_ind_st)
        # print(to_visit)
        if len(to_visit_truck) > 0:
          if tvs_ind_st > 0:
            # one or more demands on the side completed
            eng_tot += to_visit_truck[tvs_ind_st - 1][2]
          while tvs_ind_st < len(to_visit_truck):
            got[to_visit_truck[tvs_ind_st][0]] = 0
            tvs_ind_st += 1
        for i in range(ind + 2, len(to_visit), 2):
          got[to_visit[i][0]] = 0
        # jk = 0
        # for i in got:
        #   if i == 1:
        #     jk += 1
        # print("POSt unset num got set:", jk, "but num delvd", num_delvd)
        w_coeff = 1 - (truck_w / TW_DENM)
        ENG_LEVL -= to_visit[ind - 2][1] + best_eng_to_add
        # --------------------------------------
        # Drone location must be with drone_loc, 
        # next_loc must point to next demand,
        # and sel_sp is the switch point node. 
        # --------------------------------------
        # print("eng total before", eng_tot)
        eng_tot += to_visit[ind - 2][1] + best_eng_to_add + let_t[parent_loc_node][sp]
        # print("eng total after adding", to_visit[ind - 2][1] + best_eng_to_add + let_t[parent_loc_node][sp], ", is", eng_tot)
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
          # print("DONE:", truck_w, drone_loc, to_visit)
          to_visit, to_visit_truck = [], []
          break
        cycle[num_delvd] = next_dem
        num_delvd += 1
        to_visit, to_visit_truck = [], []
        # drone just came back to the truck at switch point.
        truck_loc_node = sel_sp  #  holds current selected switch point.
        # print("Entering post drone retieval section.")
        sel_sp = -1
        if demand[next_dem][1] <= MAX_WEIGHT:
          nbs, ws = [], []
          for sp in sp_poss[next_dem][1]:
            if let_t[truck_loc_node][sp] != float('inf'):
              eng_at_sp = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, sp, edges) * RECARGE_RATE)
              eng_to_add = construct_energy_launch(llep_d, next_dem, sp, edges, dedges, demand, demand[next_dem][1], DS)
              if eng_at_sp - eng_to_add > MIN_MEETUP_BATTERY_REM:
                nbs.append((sp, eng_to_add, eng_at_sp))
                ws.append((n_pherm[sp + NUM_NODES * drone_loc])**ALPHA / 
                          ((let_t[truck_loc_node][sp] / w_coeff) + eng_to_add)**BETA)
          if len(nbs) > 0:
            # print("Data: ", nbs, ws, demand[next_dem][0])
            sel_sp, eng_to_add, eng_at_sp = choices(nbs, weights=ws)[0]
            if sel_sp == next_dem:
              sel_sp = -1
        if sel_sp == -1:
          # cycle[num_delvd] = next_dem
          # num_delvd += 1
          next_dem_node = demand[next_dem][0]
          # print("eng total before", eng_tot)
          eng_tot += let_t[truck_loc_node][next_dem_node] / w_coeff
          # print("eng total after adding", let_t[truck_loc_node][next_dem_node] / w_coeff, ", is", eng_tot)
          # print("At switch point after meetin up with drone, continuing on truck", truck_w)
          truck_w -= demand[next_dem][1]
          # print("Removed", demand[next_dem][1], "to get", truck_w)
          ENG_LEVL = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, next_dem_node, edges) * RECARGE_RATE)
          truck_loc, truck_loc_node, drone_loc = next_dem, next_dem_node, -1
          swp[swp_ind] = -1
        else:
          # allocated a switch point.
          # print("eng total before", eng_tot)
          eng_tot += (let_t[truck_loc_node][sel_sp] / w_coeff)
          # print("eng total after adding", (let_t[truck_loc_node][sel_sp] / w_coeff), ", is", eng_tot)
          ENG_LEVL = eng_at_sp
          truck_loc, truck_loc_node, drone_loc = -1, -1, next_dem
          drone_w = demand[next_dem][1]
          # print("At switch point after meetin up with drone, allocating again, before:", truck_w)
          # jk = 0
          # for i in got:
          #   if i == 1:
          #     jk += 1
          # print("num got set:", jk, "but num delvd", num_delvd)
          truck_w -= DRONE_WEIGHT + drone_w
          # print("Removed", DRONE_WEIGHT, drone_w, "to get", truck_w)
          to_visit.append(sel_sp)
          to_visit.append((next_dem, eng_to_add))
          swp[swp_ind] = sel_sp
        w_coeff = 1 - (truck_w / TW_DENM)
        swp_ind += 1
      else:
        # letting truck continue and decide whether to allocate a switch point.
        # print(num_delvd)
        nbs, ws = [], []
        for i in range(DEMAND_SIZE):
          if got[i] == 0:
            nbs.append(i)
            ws.append(((n_pherm[demand[i][0] + NUM_NODES * truck_loc])**ALPHA / 
                       (let_t[truck_loc_node][demand[i][0]] / w_coeff)**BETA))
        # print(len(nbs), len(ws), truck_loc, demand[truck_loc])
        next_dem = choices(nbs, weights=ws)[0]
        got[next_dem] = 1
        cycle[num_delvd] = next_dem
        num_delvd += 1
        if demand[next_dem][1] <= MAX_WEIGHT:
          # sample a switch point, if delivery node itself then none.
          nbs, ws = [], []
          for sp in sp_poss[next_dem][1]:
            if let_t[truck_loc_node][sp] != float('inf'):
              eng_at_sp = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, sp, edges) * RECARGE_RATE)
              eng_to_add = construct_energy_launch(llep_d, next_dem, sp, edges, dedges, demand, demand[next_dem][1], DS)
              if eng_at_sp - eng_to_add > MIN_MEETUP_BATTERY_REM:
                nbs.append((sp, eng_to_add, eng_at_sp))
                ws.append((n_pherm[truck_loc * NUM_NODES + sp])**ALPHA / 
                          (let_t[truck_loc_node][sp] / w_coeff)**BETA)
          if len(nbs) > 0:
            # print("Data: ", nbs, ws, demand[next_dem][0])
            sel_sp, eng_to_add, eng_at_sp = choices(nbs, weights=ws)[0]
            if sel_sp == next_dem:
              sel_sp = -1
        if sel_sp == -1:
          # not going to be allocating a switch point.
          # cycle[num_delvd] = next_dem
          # num_delvd += 1
          next_dem_node = demand[next_dem][0]
          # print("eng total before", eng_tot)
          eng_tot += let_t[truck_loc_node][next_dem_node] / w_coeff
          # print("eng total after adding", let_t[truck_loc_node][next_dem_node] / w_coeff, ", is", eng_tot)
          ENG_LEVL = min(MAX_BATTERY_USE, ENG_LEVL + construct_time_truck(lep_t, truck_loc_node, next_dem_node, edges) * RECARGE_RATE)
          truck_loc, truck_loc_node = next_dem, next_dem_node
          # print("Found new via truck", truck_w, DRONE_WEIGHT+ sum(w[1] for w in demand))
          truck_w -= demand[next_dem][1]
          # print("Removed", demand[next_dem][1], "to get", truck_w)
          swp[swp_ind] = -2
          swp[swp_ind + 1] = -1
        else:
          # allocated a switch point.
          # print("eng total before", eng_tot)
          eng_tot += (let_t[truck_loc_node][sel_sp] / w_coeff)
          # print("eng total after adding", (let_t[truck_loc_node][sel_sp] / w_coeff), ", is", eng_tot)
          ENG_LEVL = eng_at_sp
          truck_loc, truck_loc_node, drone_loc = -1, -1, next_dem
          drone_w = demand[next_dem][1]
          # print("Truck decided to launch drone", truck_w)
          truck_w -= DRONE_WEIGHT + drone_w
          # print("Removed", DRONE_WEIGHT + drone_w, "to get", truck_w)
          to_visit.append(sel_sp)
          to_visit.append((next_dem, eng_to_add))
          swp[swp_ind] = -1
          swp[swp_ind + 1] = sel_sp
        w_coeff = 1 - (truck_w / TW_DENM)
        swp_ind += 2
      # print("Energy level: ", round(100 * ENG_LEVL / MAX_BATTERY_USE, 3))
    # -----------------------------
    # Final Deployment
    # -----------------------------
    # print("ENG begore final:", eng_tot / 10**6)
    if sel_sp == -1:
      eng_tot += let_t[truck_loc_node][src] / w_coeff
      swp[swp_ind] = -1
    else:
      eng_tot += let_t[sel_sp][src] / w_coeff
    # print(truck_w, DRONE_WEIGHT, "ENG:", eng_tot / 10**6)
    assert abs(truck_w - DRONE_WEIGHT) < 0.1, "WEIGHT ERROR"
    result.value = eng_tot
    K -= 1
    print("Ended Iteration")
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
  src = demand.pop()[0]
  ALPHA, BETA, DRONE_WEIGHT, TW_DENM = 0.9, 1.5, 12, 4535.9237
  N_PHERM_SIZE, DEMAND_SIZE = int(len(n_pherm)), len(demand)
  NUM_NODES = int(N_PHERM_SIZE / (DEMAND_SIZE + 1))
  N_PHERM_LAST = N_PHERM_SIZE - NUM_NODES
  TOTAL_WEIGHT = sum(pr[1] for pr in demand) + DRONE_WEIGHT
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
    got[next_dem] = 1
    cycle[num_delvd] = next_dem
    num_delvd += 1
    truck_loc, truck_loc_node = next_dem, demand[next_dem][0]
    eng_tot += let_t[src][truck_loc_node] / w_coeff
    truck_w -= demand[next_dem][1]
    w_coeff = 1 - (truck_w / TW_DENM)
    # -----------------------------
    while num_delvd < DEMAND_SIZE:
      nbs, ws = [], []
      for i in range(DEMAND_SIZE):
        if got[i] == 0:
          nbs.append(i)
          ws.append(((n_pherm[demand[i][0] + NUM_NODES * truck_loc])**ALPHA / 
                     (let_t[truck_loc_node][demand[i][0]] / w_coeff)**BETA))
      next_dem = choices(nbs, weights=ws)[0]
      got[next_dem] = 1
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
    assert abs(truck_w - DRONE_WEIGHT) < 0.1, "WEIGHT ERROR"
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
