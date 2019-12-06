from flatland.envs.rail_env import RailEnv
import numpy as np
import networkx as nx
import warnings

transform_dict = {
    0: (0, -1), # West
    1: (1, 0), # South
    2: (0, 1), # East
    3: (-1, 0) # North
}
# index: from direction, value to direction
valid_transition = [2, 3, 0, 1]

def transform_indexes(i, j, transition):
    tr = transform_dict[transition % 4]
    return i + tr[0], j + tr[1]

def grid2cells(rail):
    # param: env.rail.grid
    cell_transitions = []
    for i in range(rail.height):
        for j in range(rail.width):
            transitions = rail.get_full_transitions(i, j)
            bit = 1 
            for bit_ind in range(0, 16):
                if transitions & bit != 0:
                    neighbor_inds = transform_indexes(i, j, bit_ind)
                    cell_transitions.append(((i,j), neighbor_inds, convert_bit_ind_2_dir(bit_ind)))
                bit = bit << 1

    # filter dublicates
    # todo
    return cell_transitions

def convert_bit_ind_2_dir(bit_index):
    complementer = 15 - bit_index
    dir0 = complementer // 4
    dir1 = complementer % 4
    assert(dir0 >= 0 and dir1 >=0)
    if dir0 < dir1:
        return dir0, dir1, bit_index
    else:
        return dir1, dir0, bit_index

def convert_indexes_2_node(inds, width):
    return inds[0]*width + inds[1]


def convert_node_2_indexes(node, width):
    return node // width, node % width


def graph_from_cell_neighbors(cell_transitions, width, height, whitelist=None):
    if whitelist is None:
        whitelist = set()
    g = nx.Graph()
    g.add_edges_from([(convert_indexes_2_node(x, width), 
            convert_indexes_2_node(y, width),
            {'dir0':dirs[0], 'dir1':dirs[1], 'ind':dirs[2]}
        ) for x, y, dirs in cell_transitions])
    print(g.number_of_nodes(), g.number_of_edges())

    nodes2remove = set(node for node in g.nodes() if len(g[node]) == 2 and node not in whitelist)
    for node in nodes2remove:
        e = tuple(g[node])
        dirs0 = g[node][e[0]]["dir0"], g[node][e[0]]["dir1"]
        dirs1 = g[node][e[1]]["dir0"], g[node][e[1]]["dir1"]

        new_dirs = (-1, -1)
        if dirs0[0] == dirs1[0]:
            new_dirs = (dirs0[1], dirs1[1])
        elif dirs0[0] == dirs1[1]:
            new_dirs = (dirs0[1], dirs1[0])
        elif dirs0[1] == dirs1[0]:
            new_dirs = (dirs0[0], dirs1[1])
        elif dirs0[1] == dirs1[1]:
            new_dirs = (dirs0[0],dirs1[0])
        if new_dirs[0] >= 0 and new_dirs[1] >= 0:
            g.add_edge(e[0], e[1], dir0=new_dirs[0], dir1=new_dirs[1])
            g.remove_node(node)
        else:
            warnings.warn(f"Could not join edges ({node}, {e[0]}) ({node}, {e[1]}) - directions do not match")
    return g
