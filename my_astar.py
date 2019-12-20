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
    def __init__(self, point, transitions, dir=None, intersection=False):
        self.point = point
        self.parent = None
        self.H = 0
        self.G = 0
        self.nbs = transitions
        self.dir = dir
        self.intersection = intersection
    def move_cost(self,other):
        return 1

    def is_connected_to(self, cell):
        to_dir = transform_to_bit_dict[(-self.point[0] + cell[0], - self.point[1] + cell[1])]
        dir_bit = (3 - self.dir)*4 + to_dir
        mask = 1 << dir_bit
        return self.nbs & mask != 0
    
    def get_direction(self, point):
        return transition_dirs[(self.point[0] - point[0], self.point[1] - point[1])]

class AStarAgent:
    def __init__(self, grid, width, height):
        self.grid = grid
        self.width = width
        self.height = height
        self.avoid = []
            
    def children(self, point):
        x,y = point.point
        links = [self.grid[d[0]][d[1]] for d in [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y)] if 0 <= d[0] < self.height and 0 <= d[1] < self.width and d not in self.avoid]
        res = [link for link in links if point.is_connected_to(link.point)]
        is_i = len(res) > 1
        for n in res:
            n.dir = point.get_direction(n.point)
            n.intersection = is_i
        return res

    def manhattan(self, point,point2):
        return abs(point.point[0] - point2.point[0]) + abs(point.point[1]-point2.point[1])

    def aStar(self, start, goal):
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
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                return path[::-1]
            #Remove the item from the open set
            openset.remove(current)
            #Add it to the closed set
            closedset.add(current)
            #Loop through the node's children/siblings
            for node in self.children(current):
                #If it is already in the closed set, skip it
                if node in closedset:
                    continue
                #Otherwise if it is already in the open set
                if node in openset:
                    #Check if we beat the G score 
                    new_g = current.G + current.move_cost(node)
                    if node.G > new_g:
                        #If so, update the node to have a new parent
                        node.G = new_g
                        node.parent = current
                else:
                    #If it isn't in the open set, calculate the G and H score for the node
                    node.G = current.G + current.move_cost(node)
                    node.H = self.manhattan(node, goal)
                    #Set the parent to our current item
                    node.parent = current
                    #Add it to the set
                    openset.add(node)
        # If there is no path
        return None

    def add_cell_to_avoid(self, cell):
        self.avoid.append(cell)

    def remove_cell_from_avoid(self, cell):
        self.avoid.remove(cell)