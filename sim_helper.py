from enlib import EnergyHelper
import matplotlib.pyplot as plt

def sample_energy_fcn(vert1, vert2):
    pass

def convert_to_adj_list(nodes, edges):
    adj_list = []
    for edge_list in edges:
        adj = []
        for adjacent_node in edge_list:
            adj.append(adjacent_node[0])
        adj_list.append(adj)
    return adj_list

def plot_graph_adj(nodes, adj_list):
    for edge_list in enumerate(adj_list):
        for edge in edge_list[1]:
            x = [nodes[edge_list[0]][0], nodes[edge][0]]
            y = [nodes[edge_list[0]][1], nodes[edge][1]]
            plt.plot(x, y, 'b')
    # plt.show()
    x=[]
    y=[]
    for p in nodes:
        x.append(p[0])
        y.append(p[1])
    plt.scatter(x, y)