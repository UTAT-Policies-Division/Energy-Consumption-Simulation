import matplotlib.pyplot as plt

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
    filename = "set-" + heading[4] + "-" + heading[22:heading.index(" ", 22)] + "ms.png"
    plt.legend(loc="best")
    plt.savefig("plots/{}".format(filename), dpi=200)
    plt.cla()
    plt.clf()

def compile_specific(filename):
    lines = open(filename, 'r').readlines()
    values = []
    for line in filename:
        if len(line) == 0 or line[0] != 'U':
            continue
        start = line.index(":", line.index(":") + 1) + 2
        end = line.index(" ", start)
        values.append(round(float(line[start:end]) / 3.6, 2))
    return values

def compile_seq(filename):
    lines = open(filename, 'r').readlines()
    
    truck_values, drone_values, current_values = None, None, None
    heading = None
    seekingCompletion = False
    
    for line in lines:
        line = line.strip()
        if len(line) == 0:
            continue
        if line.startswith("Set"):
            if seekingCompletion:
                plot_eng(heading, drone_values, "truck + drone", truck_values, "truck only")
                seekingCompletion = False
            else:
                heading = line[0:line.index("b") - 1]
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

compile_seq("data/speed-results.txt")
