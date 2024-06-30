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
    for values in [all_values[i:i + set_size] for i in range(0, len(all_values), set_size)]:
        ax.plot(iters, values, label=labels[j])
        ax.plot(iters, [values[-1] for _ in range(len(iters))], alpha=0.4, linestyle="dotted")
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
file_list = ["data/sets-234-regen.txt", "data/set-2-run-2.txt", "data/set-2-run-3.txt"]

aggregator(file_list)

# all_values = compile_all("special.txt")
# all_val_sep = [all_values[i:i + 300] for i in range(0, len(all_values), 300)]
# cnt = 0
# for min_w in [0.0, 1.0]:
#     for V in [10, 20]:
#         plot_all(all_val_sep[cnt], 100, ["Truck Only", "BVLOS Truck + Drone", "VLOS Truck + Drone"], 
#                  "Set {} - {} kg min weight - {} m/s".format(2, min_w, V), True)
#         cnt += 1

# plot_all(compile_all("cmp.txt"), 100, ["Static q = 10**6", 
#                                        "First half only exploration promotion via 0.05", 
#                                        "First half 0.05, second half demotion via 0.015", 
#                                        "Quarterly exploration promotion & demotion"], "Q Hyperparamter Systems")
