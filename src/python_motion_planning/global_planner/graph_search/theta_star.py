"""
@file: theta_star.py
@breif: Theta* motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils import Env, Node, Grid


class ThetaStar(AStar):
    """
    Class for Theta* motion planning.

    Parameters:
        start (tuple):
            start point coordinate
        goal (tuple):
            goal point coordinate
        env (Grid):
            environment
        heuristic_type (str):
            heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.ThetaStar((5, 5), (45, 25), pmp.Grid(51, 31))
        >>> cost, path, expand = planner.plan()
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Theta*: Any-Angle Path Planning on Grids
        [2] Any-angle path planning on non-uniform costmaps
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)


    def __str__(self) -> str:
        return "Theta*"

    def plan(self) -> tuple:
        """
        Theta* motion plan function.

        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
        """
        # OPEN list (priority queue) and CLOSED list (hash table)
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            # exists in CLOSED list
            if node.current in CLOSED:
                continue

            # goal found
            if node == self.goal:
                print("Found goal")
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):                
                # exists in CLOSED list
                if node_n.current in CLOSED:
                    continue
                
                # path1: set initial parent and costs
                node_n.parent = node.current
                node_n.g = node.g + self.dist(node, node_n)
                node_n.h = self.h(node_n, self.goal)

                # path2: check for line-of-sight to grandparent
                node_p = CLOSED.get(node.parent)
                if node_p:
                    self.updateVertex(node_p, node_n)

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break
                
                # update OPEN list
                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node
        return [], [], []

    def extractPath(self, closed_list: dict) -> tuple:
        """
        Extract the path based on the CLOSED list.
        For Theta*, this includes line-of-sight segments.

        Parameters:
            closed_list (dict): CLOSED list

        Returns:
            cost (float): the cost of planned path
            path (list): the planning path
        """
        cost = 0
        node = closed_list[self.goal.current]
        path = [node.current]
        
        while node.current != self.start.current:
            if node.parent in closed_list:
                node_parent = closed_list[node.parent]
                cost += self.dist(node, node_parent)
                node = node_parent
                path.append(node.current)
            else:
                break
        
        path.reverse()
        return cost, path

    def updateVertex(self, node_p: Node, node_c: Node) -> None:
        """
        Update extend node information with current node's parent node.

        Parameters:
            node_p (Node): parent node
            node_c (Node): current node
        """
        if self.lineOfSight(node_c, node_p):
            # path 2
            if node_p.g + self.dist(node_c, node_p) <= node_c.g:
                node_c.g = node_p.g + self.dist(node_c, node_p)
                node_c.parent = node_p.current

    def lineOfSight(self, node1: Node, node2: Node) -> bool:
        """
        Judge collision when moving from node1 to node2 using 3D line-of-sight check.

        Parameters:
            node1 (Node): start node
            node2 (Node): end node

        Returns:
            line_of_sight (bool): True if line of sight exists ( no collision ) else False
        """


        self.obstacles = self.obstacles.union(self.env.inner_obstacles)
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return False
        
        x1, y1, z1 = node1.current
        x2, y2, z2 = node2.current

        # Check bounds
        if (x1 < 0 or x1 >= self.env.x_range or y1 < 0 or y1 >= self.env.y_range or 
            z1 < 0 or z1 >= self.env.z_range):
            return False
        if (x2 < 0 or x2 >= self.env.x_range or y2 < 0 or y2 >= self.env.y_range or 
            z2 < 0 or z2 >= self.env.z_range):
            return False

        # 3D Bresenham-like line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dz = abs(z2 - z1)
        
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1
        sz = 1 if z2 > z1 else -1
        
        x, y, z = x1, y1, z1
        
        # Determine dominant axis
        if dx >= dy and dx >= dz:
            # X is dominant
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while x != x2:
                if p1 >= 0:
                    y += sy
                    p1 -= 2 * dx
                if p2 >= 0:
                    z += sz
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
                x += sx
                if (x, y, z) in self.obstacles:
                    return False
        elif dy >= dx and dy >= dz:
            # Y is dominant
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while y != y2:
                if p1 >= 0:
                    x += sx
                    p1 -= 2 * dy
                if p2 >= 0:
                    z += sz
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
                y += sy
                if (x, y, z) in self.obstacles:
                    return False
        else:
            # Z is dominant
            p1 = 2 * dx - dz
            p2 = 2 * dy - dz
            while z != z2:
                if p1 >= 0:
                    x += sx
                    p1 -= 2 * dz
                if p2 >= 0:
                    y += sy
                    p2 -= 2 * dz
                p1 += 2 * dx
                p2 += 2 * dy
                z += sz
                if (x, y, z) in self.obstacles:
                    return False
        
        # Special debug for the problematic path
        if (node1.current == (18, 10, 1) and node2.current == (1, 17, 1)) or \
           (node1.current == (1, 17, 1) and node2.current == (18, 10, 1)):
            # Sample a few points along the line to check
            for i in range(5):
                t = i / 4.0  # 0, 0.25, 0.5, 0.75, 1.0
                x = int(x1 + t * (x2 - x1))
                y = int(y1 + t * (y2 - y1))
                z = int(z1 + t * (z2 - z1))
                is_obstacle = (x, y, z) in self.obstacles
              
        
        return True
