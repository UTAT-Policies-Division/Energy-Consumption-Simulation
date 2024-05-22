from enlib import EnergyHelper

def sample_energy_fcn(vert1, vert2):
    pass

def convert_to_adj_list(graph: EnergyHelper):
    edges = graph.edges
    vertices = graph.nodes

    point_count = max(edges.keys())
    adj_matrix = []

    verts = [*vertices.values()]

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


