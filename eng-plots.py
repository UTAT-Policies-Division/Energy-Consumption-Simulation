import matplotlib.pyplot as plt

def get_delivery_stats(swp):
    num_drone, num_side = 0, 0
    if swp[0] >= 0:
      num_drone += 1
    j = 1
    while j < len(swp) - 1:
      if swp[j] == -2:
        if swp[j+1] == -2:
          num_side += 1
        elif swp[j+1] >= 0:
          num_drone += 1
      elif swp[j+1] >= 0:
        num_drone += 1
      j += 2
    return (num_drone, num_side)

def plot_eng(heading, valuesA, labelA, valuesB, labelB):
    iters = [i for i in range(1, len(valuesB) + 1)]
    _, ax = plt.subplots()
    ax.plot(iters, valuesA, color="green", label=labelA)
    last_val = valuesA[-1]
    ax.text(-3, last_val, "{}".format(last_val), fontsize='small')
    # ax.set_yticks(np.append(ax.get_yticks(), last_val))
    ax.plot(iters, [last_val for _ in range(len(iters))], color="red", alpha=0.4, linestyle="dotted")
    ax.plot(iters, valuesB, color="blue", label=labelB)
    last_val = valuesB[-1]
    ax.text(7, last_val, "{}".format(last_val), fontsize='small')
    # ax.set_yticks(np.append(ax.get_yticks(), last_val))
    ax.plot(iters, [last_val for _ in range(len(iters))], color="magenta", alpha=0.4, linestyle="dotted")
    if valuesB[-1] > valuesA[-1]:
        print("Drone Wins!")
    else:
        print("Truck Wins :(")
    plt.xlabel("Number of Iterations")
    plt.ylabel("Energy Consumption (kWh)")
    plt.title(heading)
    filename = "set-" + heading[4] + "-" + heading[28:heading.index(" ", 28)] + "ms-" + heading[8] + heading[10] + ".png"
    plt.legend(loc="best")
    plt.savefig("plots/{}".format(filename), dpi=200)
    plt.cla()
    plt.clf()

def plot_all(all_values, set_size, labels, heading, extract_filename=False):
    assert len(all_values) / set_size == len(labels)
    _, ax = plt.subplots()
    iters = [i for i in range(1, set_size + 1)]
    j = 0
    k = -5
    for values in [all_values[i:i + set_size] for i in range(0, len(all_values), set_size)]:
        ax.plot(iters, values, label=labels[j])
        ax.plot(iters, [values[-1] for _ in range(len(iters))], alpha=0.4, linestyle="dotted")
        ax.text(k, values[-1], "{}".format(values[-1]), fontsize='small')
        k += 10
        j += 1
    plt.xlabel("Number of Iterations")
    plt.ylabel("Energy Consumption (kWh)")
    plt.title(heading)
    plt.legend(loc="best")
    if extract_filename:
        filename = "set-" + heading[4] + "-" + heading[28:heading.index(" ", 28)] + "ms-" + heading[8] + heading[10] + ".png"
    else:
        filename = "result.png"
    plt.savefig(filename, dpi=200)
    plt.cla()
    plt.clf()

def compile_all(filename):
    lines = open(filename, 'r').readlines()
    values = []
    for line in lines:
        if len(line) == 0 or line[0] != 'U':
            continue
        start = line.index(":", line.index(":") + 1) + 2
        end = line.index(" ", start)
        values.append(round(float(line[start:end]) / 3.6, 2))
    return values

def aggregator_raw(all_values, split_size):
    all_val_sep = [all_values[i:i + split_size] for i in range(0, len(all_values), split_size)]
    num = len(all_val_sep)
    collec = [0 for _ in range(split_size)]
    for all_values in all_val_sep:
        for i in range(split_size):
            collec[i] += all_values[i]
    for i in range(split_size):
        collec[i] = round(collec[i] / num, 2)
    plot_all(collec, 100, ["Truck Only", "BVLOS Truck + Drone", "VLOS Truck + Drone"], 
                 "Set {} - {} kg min weight - {} m/s".format(2, 1.0, 20), True)

def aggregator(file_list):
   agg = {}
   for fname in file_list:
      lines = open(fname, 'r').readlines()

      truck_values, drone_values, current_values = None, None, None
      heading, lastLine = None, None
      seekingCompletion = False
      
      for line in lines:
          line = line.strip()
          if len(line) == 0:
              continue
          if line.startswith("Set"):
              if seekingCompletion:
                  dr_del, sd_del = get_delivery_stats(list(map(int, lastLine[1:-1].split(','))))
                  for i in range(100):
                      agg[heading][0][i] += drone_values[i]
                      agg[heading][1][i] += truck_values[i]
                  agg[heading][2] += dr_del
                  agg[heading][3] += sd_del
                  agg[heading][4] += 1
                  seekingCompletion = False
              else:
                  set_num = line[0:5]
                  set_minw = str(float(line[21:line.index(" ", 21)])) + " kg min weight"
                  set_ds = line[line.index(" ", line.index("b") - 9) + 1:line.index("b") - 1]
                  heading = set_num + " - " + set_minw + " - " + set_ds
                  print(heading)
                  if heading not in agg:
                      agg[heading] = [[0 for _ in range(100)], [0 for _ in range(100)], 0, 0, 0]
                  seekingCompletion = True
          elif line.startswith("Truck Only:"):
              truck_values = []
              current_values = truck_values
          elif line.startswith("Truck + Drone:"):
              drone_values = []
              current_values = drone_values
          elif line[0] == 'U':
              start = line.index(":", line.index(":") + 1) + 2
              end = line.index(" ", start)
              current_values.append(round(float(line[start:end]) / 3.6, 2))
          lastLine = line
   for k in agg:
       num = agg[k][-1]
       if num == 1:
           continue
       for i in range(100):
           agg[k][0][i] = round(agg[k][0][i] / num, 2)
           agg[k][1][i] = round(agg[k][1][i] / num, 2)
       agg[k][2] = round(agg[k][2] / num, 2)
       agg[k][3] = round(agg[k][3] / num, 2)
       plot_eng(k, agg[k][0], "truck + drone", agg[k][1], "truck only")
       print(k, "Num drone deliveries, num side deliveries:", (agg[k][2], agg[k][3]))

def compile_seq(filename):
    lines = open(filename, 'r').readlines()
    
    truck_values, drone_values, current_values = None, None, None
    heading, lastLine = None, None
    seekingCompletion = False
    
    for line in lines:
        line = line.strip()
        if len(line) == 0:
            continue
        if line.startswith("Set"):
            if seekingCompletion:
                print("Num drone deliveries, num side deliveries:", get_delivery_stats(list(map(int, lastLine[1:-1].split(',')))))
                plot_eng(heading, drone_values, "truck + drone", truck_values, "truck only")
                seekingCompletion = False
            else:
                set_num = line[0:5]
                set_minw = str(float(line[21:line.index(" ", 21)])) + " kg min weight"
                set_ds = line[line.index(" ", line.index("b") - 9) + 1:line.index("b") - 1]
                heading = set_num + " - " + set_minw + " - " + set_ds
                print(heading)
                seekingCompletion = True
        elif line.startswith("Truck Only:"):
            truck_values = []
            current_values = truck_values
        elif line.startswith("Truck + Drone:"):
            drone_values = []
            current_values = drone_values
        elif line[0] == 'U':
            start = line.index(":", line.index(":") + 1) + 2
            end = line.index(" ", start)
            current_values.append(round(float(line[start:end]) / 3.6, 2))
        lastLine = line


# compile_seq("data/sets-234-main.txt")
# file_list = ["data/sets-234-regen.txt", "run1.txt"]

# aggregator(["data/new/base-case.txt", "data/new/spd-wht.txt", "data/new/set-345-base.txt"])
# aggregator(file_list)

aggregator_raw(compile_all("data/new/vlos.txt"), 300)

# all_values = compile_all("vlos-main1.txt")
# print(len(all_values))
# all_val_sep = [all_values[i:i + 300] for i in range(0, len(all_values), 300)]
# cnt = 0
# for min_w in [1.0, 1.0, 1.0]:
#     for V in [20, 10, 20]:
#         plot_all(all_val_sep[cnt], 100, ["Truck Only", "BVLOS Truck + Drone", "VLOS Truck + Drone"], 
#                  "Set {} - {} kg min weight - {} m/s".format(2, min_w, V), True)
#         cnt += 1

# plot_all(compile_all("cmp.txt"), 100, ["Static q = 10**6", 
#                                        "First half only exploration promotion via 0.05", 
#                                        "First half 0.05, second half demotion via 0.015", 
#                                        "Quarterly exploration promotion & demotion"], "Q Hyperparamter Systems")

# swp_strings = ["[-1, -1, 2690, 2691, -1, -1, 1077, 1231, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 345, 1012, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 549, 2790, -1, -2, -1, -2, -1, -1, 4581, 947, -1, -1, 947, 255, -1, -2, -1, -2, -1, -1, 964, 3814, -1, -2, -1, -1, 5152, -2, -2, 3554, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3663, 4392, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1360, 3627, -1, -1, 44, -2, 13, 111, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 5125, 1580, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4848, -2, -2, 1984, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1202, 1829, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2275, 1583, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1261, 3177, -1, -1, 3177, 1906, -1, -2, -1, -2, -1, -1, 1279, 3215, -1, -2, -1, -1, 3673, 3103, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3860, -2, -2, 3155, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 910, 902, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3112, 3925, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1]",
# "[-1, -2, -1, -1, 2690, 1820, 1820, 2690, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 164, 160, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2119, 2120, -1, -2, -1, -2, -1, -2, -1, -1, 3813, 977, -1, -1, 964, 962, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 5462, 4750, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4391, 1763, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 6048, 1829, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3525, 5652, -1, -2, -1, -1, 2413, 2381, 1731, 1730, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 467, 3257, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 5371, 3838, -1, -1, 2417, 1629, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 105, 1643, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 107, 3112, -1, -2, -1, -2, -1, -1, 894, 1155, -1, -2, -1, -2, -1, -1, 2610, 4361, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1471, 749, -1, -1]",
# "[-1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1056, 520, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1948, -2, -2, 316, -1, -2, -1, -1, 995, 4961, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 950, 3711, -1, -1, 909, 2266, -1, -2, -1, -1, 1723, 9, -1, -1, 9, 921, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3436, 3459, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4774, -2, 5522, 3396, -1, -2, -1, -1, 3308, 3927, -1, -2, -1, -2, -1, -2, -1, -1, 2988, 3449, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1365, 3084, -1, -2, -1, -2, -1, -2, -1, -1, 639, 604, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 104, 1643, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 867, 47, -1, -2, -1, -2, -1, -2, -1, -1, 1332, 4544, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4212, 3055, -1, -2, -1, -2, -1, -2, -1, -1, 300, 4640, -1, -2, -1, -1, 3986, 4201, -1, -2, -1, -1, 4603, 4567, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2244, -2, 6372, -2, -2, -2, -2, 3289, -1, -2, -1, -1, 2261, 405, -1, -2, -1, -2, -1, -2, -1, -1, 1290, 4148, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2841, 2843, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 183, 1526, -1, -2, -1, -2, -1, -2, -1, -1, 2782, 1950]",
# "[-1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1056, 526, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 311, 314, -1, -2, -1, -1, 1941, 1940, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2305, 3623, -1, -2, -1, -2, -1, -1, 61, 66, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 9, 10, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3522, 3522, -1, -2, -1, -2, -1, -1, 2988, 3005, -1, -2, -1, -1, 3004, 4333, 4333, 2400, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2926, 3082, -1, -2, -1, -2, -1, -1, 2937, 1332, -1, -1, 2054, 2055, -1, -2, -1, -2, -1, -2, -1, -1, 4212, 3427, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1575, 1643, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 26, 1662, 52, 47, -1, -2, -1, -1, 107, 2386, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3240, 3507, -1, -1, 3986, 3239, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2659, 5341, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2852, 1830, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3385, 3257, -1, -1, 468, 465, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1]",
# "[-1, -2, -1, -2, -1, -2, -1, -1, 486, 3832, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 985, 969, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3548, 5527, -1, -1, 5527, 4249, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1252, 2748, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4231, -2, 2188, 259, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1665, 113, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 29, 3760, -1, -2, -1, -2, -1, -2, -1, -1, 1768, 734, -1, -2, -1, -2, -1, -2, -1, -1, 2329, 4174, -1, -1, 4174, 4165, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 5692, 2704, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2912, 2840, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 1930, 1356, -1, -2, -1, -1, 1962, 801, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3206, 2945, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 608, 1641, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3978, 3429, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4742, 5162, -1, -2, -1, -1, 3484, 2788, -1, -2, -1, -2, -1, -2, -1, -1, 1940, 344, -1, -1]",
# "[-1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2179, 2882, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2229, 2128, -1, -1, 971, 970, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 5527, 5424, -1, -2, -1, -1, 5087, 5087, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2188, 1241, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 884, 122, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 3758, 1654, -1, -2, -1, -1, 3586, 3972, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2851, 2854, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 5047, 4456, -1, -2, -1, -2, -1, -2, -1, -1, 4850, 3428, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 608, 605, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 2389, 4038, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -2, -1, -1, 4434, 2454, -1, -2, -1, -2, -1, -1]"]

avg_dron_del, avg_side_del, cnt = 0, 0, 0
for i in range(0, len(swp_strings), 2):
    dron_del, side_del = get_delivery_stats(list(map(int, swp_strings[i][1:-1].split(','))))
    avg_dron_del += dron_del
    avg_side_del += side_del
    cnt += 1
print("BVLOS", avg_dron_del / cnt, avg_side_del / cnt)

avg_dron_del, avg_side_del, cnt = 0, 0, 0
for i in range(1, len(swp_strings), 2):
    dron_del, side_del = get_delivery_stats(list(map(int, swp_strings[i][1:-1].split(','))))
    avg_dron_del += dron_del
    avg_side_del += side_del
    cnt += 1
print("VLOS", avg_dron_del / cnt, avg_side_del / cnt)

