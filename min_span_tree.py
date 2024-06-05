"""This File contains an implementation of the Krusal Algorithm 
for calculating an MST from a graph."""

import copy
import math
import random
import matplotlib.pyplot as plt
from progress.bar import Bar

#################################################################
# Expected Graph Format:
# vertices: A List of Vertices, vertex type does not matter; fed into energy function
# edges: An array of lists, listing the index connections of vertices of that index
#################################################################
def kruskal_mst(vertices: list, edges: list, _start_vertex, weight: callable):
    """Implementation of the krusal algorithm for generating Minimum Spanning Trees"""

    v_len = len(vertices)
    mst_e = [[] for _ in range(v_len)]
    total_cost = 0

    edge_pairs = []
    for i in range(v_len):
        for adj in edges[i]:
            edge_pairs.append([weight(vertices[i], vertices[adj]), i, adj])

    sorted_edges = sorted(edge_pairs)

    with Bar(
        "Processing...", max=len(edge_pairs), suffix="%(percent).2f%% - %(eta)ds"
    ) as progress_bar:
        while len(sorted_edges) > 0:

            edge, cost = sorted_edges[0][1:3:1], sorted_edges[0][0]
            sorted_edges.pop(0)

            if not is_cyclic(mst_e, edge, v_len):
                mst_e[edge[0]].append(edge[1])
                mst_e[edge[1]].append(edge[0])
                total_cost += cost
            progress_bar.next()

    return mst_e, total_cost


def is_cyclic(edge_pairs: list, test_edge: tuple, vertex_count: int):
    """Checks the graph for any cyclic paths"""
    edges = copy.deepcopy(edge_pairs)
    stack = [test_edge[0], test_edge[1]]
    visited = [False] * vertex_count

    while stack:

        popped = stack[-1]
        stack.pop()

        if visited[popped]:
            return True

        visited[popped] = True

        stack.extend(edges[popped])
        for adj in edges[popped]:
            edges[adj].remove(popped)
        edges[popped] = []

    return False


def euclidean_distance(vector_a, vector_b):
    """Returns the Euclidean Distance between vectors v1 and v2"""
    return math.sqrt(
        (vector_a[0] - vector_b[0]) ** 2 + (vector_a[1] - vector_b[1]) ** 2
    )


def grid(n_x: int, n_y: int, rand=True):
    """Returns a Grid with dimensions nx X ny, with options for randomization"""
    vertices = []
    edges = []
    for i in range(n_x):
        for j in range(n_y):
            if rand:
                vertices.append((random.random(), random.random()))
            else:
                vertices.append((i, j))

    for i in range(n_x * n_y):
        adj = []
        distances = []
        for j in range(n_x * n_y):
            distances.append(euclidean_distance(vertices[i], vertices[j]))

        sorted_distances = sorted(distances)

        for j in sorted_distances[0 : n_x * n_y // 100 : 1]:
            index = distances.index(j)
            adj.append(index)

        edges.append(adj)

    return vertices, edges


def preorder_traversal(edges: list, start_index: int, v_len: int):
    """Returns the list of vertices in DFS Search"""
    visited = [False] * v_len
    stack = [start_index]
    tour = []

    while stack:
        top = stack.pop(0)

        if not visited[top]:
            tour.append(top)
            stack[0:0] = edges[top]
            visited[top] = True
    tour.append(start_index)
    return tour


def plot_mst(verts: list, edges: list, save_plot=False, name=None):
    """Plots the Edge Pair list of a Minimum Spanning Tree"""
    for edge in enumerate(edges):
        for adj in edge[1]:
            plt.plot(
                [verts[edge[0]][0], verts[adj][0]], [verts[edge[0]][1], verts[adj][1]]
            )
    points_x = []
    points_y = []
    for vert in verts:
        points_x.append(vert[0])
        points_y.append(vert[1])
    plt.scatter(points_x, points_y, s=5)

    if save_plot:
        plt.savefig(f"mst_imgs/{name}.png")
    else:
        # pass
        plt.show()



def tsp_krusal_solve(vertices, adj_list, start, energy):
    mst, _mst_cost = kruskal_mst(vertices, adj_list, start, energy)
    t = preorder_traversal(mst, start, len(vertices))
    return t


