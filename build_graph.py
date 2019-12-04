from flatland.envs.rail_env import RailEnv
import numpy as np
import networkx as nx

transform_dict = {
    0: (0, -1), # West
    1: (1, 0), # South
    2: (0, 1), # East
    3: (-1, 0) # North
}

def transform_indexes(i, j, transition):
    tr = transform_dict[transition % 4]
    return i + tr[0], j + tr[1]

# env.rail.grid
def grid2cells(rail):
    cell_transitions = []
    for i in range(rail.height):
        for j in range(rail.width):
            transitions = rail.get_full_transitions(i, j)
            bit = 1 << 8
            for bit_ind in range(8, 16):
                if transitions & bit != 0:
                    neighbor_inds = transform_indexes(i, j, bit_ind)
                    cell_transitions.append(((i,j), neighbor_inds))
                bit = bit << 1
            # for tr in transitions:
                # pass
            
            # for bit_ind, bit in enumerate(np.binary_repr(grid[i, j])):
            #     if bit == '1':
    return cell_transitions

def convert_indexes_2_node(inds, width):
    return inds[0]*width + inds[1]

def convert_node_2_indexes(node, width):
    return node // width, node % width

def graph_from_cell_neighbors(cell_transitions, width, height, whitelist=None):
    if whitelist is None:
        whitelist = set()
    g = nx.Graph()
    # for edges in cell_transitions:
    # g.add_edges_from(map(lambda x, y: (x[0]*width + x[1],x[0]*width + x[1]), cell_transitions))
    g.add_edges_from([(convert_indexes_2_node(x, width), convert_indexes_2_node(y, width)) for x, y in cell_transitions])
    print(g.number_of_nodes(), g.number_of_edges())

    nodes2remove = set(node for node in g.nodes() if len(g[node]) == 2 and node not in whitelist)
    for node in nodes2remove:
        e = tuple(g[node])
        g.add_edge(e[0], e[1])
        g.remove_node(node)
    return g