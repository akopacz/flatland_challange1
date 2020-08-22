import numpy as np

transform_to_bit_dict = {
    (0, -1):0, # West
    (1, 0):1, # South
    (0, 1):2, # East
    (-1, 0):3 # North
}

transition_dirs = {
    (0, -1):1, # East
    (-1, 0):2, # South
    (0, 1):3, # West
    (1, 0):0 # North
}

class Node:
    def __init__(self, point, transitions, dir=None, intersection=False, starting_timestamp=0, speed=1.0):
        self.point = point
        self.parent = None
        self.H = 0
        self.G = 0
        self.timestamp = starting_timestamp
        self.nbs = int(transitions)
        self.dir = dir
        self.intersection = intersection
        self.move_cost = speed

    def move_cost(self,other=None):
        return self.move_cost

    def is_connected_to(self, cell):
        to_dir = transform_to_bit_dict[(-self.point[0] + cell[0], - self.point[1] + cell[1])]
        dir_bit = (3 - self.dir)*4 + to_dir
        mask = 1 << dir_bit
        
        # ignore agent direction, check if neighbor cell can be accessed
        # mask = 4369 << to_dir
        return self.nbs & mask != 0
    
    # def is_connected_to(self, cell):
    #     target_cell_direction = transform_to_bit_dict[(-self.point[0] + cell[0], - self.point[1] + cell[1])]
        
    
    # def get_neighbors(self):
    #     links = [self.grid[d[0]][d[1]] for d in [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y)] if 0 <= d[0] < self.height and 0 <= d[1] < self.width ]
    #     return [link for link in links if point.is_connected_to(link.point)]
    
    def get_direction(self, point):
        return transition_dirs[(self.point[0] - point[0], self.point[1] - point[1])]

class AStarAgent:
    def __init__(self, grid, width, height):
        self.grid = grid
        self.width = width
        self.height = height
        # self.avoid = []
        self.last_visited = np.empty((height, width, 2), dtype=int)
        self.last_visited[:] = -10
            
    def get_neighbors(self, node, agent_id, t_stamp):
        x,y = node.point
        links = [self.grid[d[0]][d[1]] for d in [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y)] if 0 <= d[0] < self.height and 0 <= d[1] < self.width and node.is_connected_to(d)]
        for link in links:
            if abs(self.last_visited[link.point[0], link.point[1], 0] - t_stamp) <= 2 and self.last_visited[link.point[0], link.point[1], 1] != agent_id:
                print(link.point)
        return [link for link in links if abs(self.last_visited[link.point[0], link.point[1], 0] - t_stamp) > 2 or self.last_visited[link.point[0], link.point[1], 1] == agent_id]
    
    def children(self, point, agent_id, move_cost):
        res = self.get_neighbors(point, agent_id, t_stamp=point.timestamp + move_cost)
        is_i = len(res) > 1
        for n in res:
            n.dir = point.get_direction(n.point)
            n.intersection = is_i
        return res

    def manhattan(self, point, point2):
        return abs(point.point[0] - point2.point[0]) + abs(point.point[1]-point2.point[1])

    def aStar(self, start, goal, agent_id, agent_speed=1.0):
        move_cost = int(1/agent_speed)
        #The open and closed sets
        openset = set()
        closedset = set()
        #Current point is the starting point
        current = start
        #Add the starting point to the open set
        openset.add(current)
        #While the open set is not empty

        while openset:
            #Find the item in the open set with the lowest G + H score
            current = min(openset, key=lambda o:o.G + o.H)
            #If it is the item we want, retrace the path and return it
            if current.point == goal.point:
                # Construct path
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                self.update_timestamps(path, agent_id)
                return path[::-1]
            #Remove the item from the open set
            openset.remove(current)
            #Add it to the closed set
            closedset.add(current)
            #Loop through the node's children/siblings
            for node in self.children(current, agent_id, move_cost):
                #If it is already in the closed set, skip it
                if node in closedset:
                    continue
                #Otherwise if it is already in the open set
                if node in openset:
                    #Check if we beat the G score
                    # new_g = current.G + current.move_cost(node)
                    new_g = current.G + move_cost
                    if node.G > new_g:
                        #If so, update the node to have a new parent
                        node.G = new_g
                        node.parent = current
                        # node.timestamp = current.timestamp + current.move_cost( )
                        node.timestamp = current.timestamp + move_cost
                        node.dir = transition_dirs[current.point[0] - node.point[0], current.point[1] - node.point[1]]
                else:
                    #If it isn't in the open set, calculate the G and H score for the node
                    # node.G = current.G + current.move_cost()
                    node.G = current.G + move_cost
                    node.H = self.manhattan(node, goal)
                    # node.timestamp = current.timestamp + current.move_cost()
                    node.timestamp = current.timestamp + move_cost

                    node.dir = transition_dirs[current.point[0] - node.point[0], current.point[1] - node.point[1]]
                    #Set the parent to our current item
                    node.parent = current
                    #Add it to the set
                    openset.add(node)
        # If there is no path
        return None

    def update_timestamps(self, path, agent_id):
        for node in path:
            self.last_visited[node.point[0], node.point[1], 0] = node.timestamp
            self.last_visited[node.point[0], node.point[1], 1] = agent_id


    def visited_node(self, position, timestamp, agent_id):
        self.last_visited[position[0], position[1], 0] = timestamp
        self.last_visited[position[0], position[1], 1] = agent_id

    # def add_cell_to_avoid(self, cell):
    #     self.avoid.append(cell)

    # def empty_avoid(self):
    #     self.avoid = []
    
    # def remove_cell_from_avoid(self, cell):
    #     self.avoid.remove(cell)

