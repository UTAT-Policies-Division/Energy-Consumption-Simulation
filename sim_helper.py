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

def plot_tour(nodes, tour, point_count, edge_color='green', node_color='green'):
    for i in range(point_count):
        n0 = nodes[tour[i]]
        n1 = nodes[tour[i+1]]

        x = [n0[0], n1[0]]
        y = [n0[1], n1[1]]

        plt.plot(x, y, color=edge_color)
        plt.scatter(x, y, color=node_color)

def plot_graph_adj(nodes, adj_list):
    x=[]
    y=[]
    for p in nodes:
        x.append(p[0])
        y.append(p[1])
    plt.scatter(x, y, color='silver')

    for edge_list in enumerate(adj_list):
        for edge in edge_list[1]:
            x = [nodes[edge_list[0]][0], nodes[edge][0]]
            y = [nodes[edge_list[0]][1], nodes[edge][1]]
            plt.plot(x, y, 'lightgrey')
    # plt.show()