from enlib import EnergyHelper
import matplotlib.pyplot as plt

def sample_energy_fcn(vert1, vert2):
    pass

def convert_to_adj_list(graph: EnergyHelper):
    edges = graph.edges
    vertices = graph.nodes

    point_count = len(vertices)
    adj_matrix = []

    verts = vertices

    for i in range(point_count):
        connected = edges[i]
        adj = []
        for edge in connected:
            try:
                edge_index = verts.index(edge)
                adj.append(edge_index)
            except(ValueError):
                pass

        adj_matrix.append(adj)

        # print(adj)
    return adj_matrix


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

def plot_tour(nodes, tour, point_count, edge_color='green', node_color='green'):
    for i in range(point_count):
        n0 = nodes[tour[i]]
        n1 = nodes[tour[i+1]]

        x = [n0[0], n1[0]]
        y = [n0[1], n1[1]]

        plt.plot(x, y, color=edge_color)
        plt.scatter(x, y, color=node_color)