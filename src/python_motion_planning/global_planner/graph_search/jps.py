"""
@file: jps.py
@breif: Jump Point Search motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils import Env, Node, Grid

class JPS(AStar):
    """
    Class for JPS motion planning.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.JPS((5, 5), (45, 25), pmp.Grid(51, 31))
        >>> cost, path, expand = planner.plan()     # planning results only
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Online Graph Pruning for Pathfinding On Grid Maps
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
    
    def __str__(self) -> str:
        return "JPS"

    def plan(self) -> tuple:
        """
        JPS motion plan function.

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
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            jp_list = []
            for motion in self.motions:
                jp = self.jump(node, motion)
                # exists and not in CLOSED list
                if jp and jp.current not in CLOSED:
                    jp.parent = node.current
                    jp.h = self.h(jp, self.goal)
                    jp_list.append(jp)

            for jp in jp_list:
                # update OPEN list
                heapq.heappush(OPEN, jp)

                # goal found
                if jp == self.goal:
                    break

            CLOSED[node.current] = node
        return [], [], []

    def jump(self, node: Node, motion: Node):
        """
        Jumping search recursively for 3D space.

        Parameters:
            node (Node): current node
            motion (Node): the motion that current node executes

        Returns:
            jump_point (Node): jump point or None if searching fails
        """
        # explore a new node
        new_node = node + motion
        new_node.parent = node.current
        new_node.h = self.h(new_node, self.goal)

        # hit the obstacle
        if new_node.current in self.obstacles:
            return None

        # goal found
        if new_node == self.goal:
            return new_node

        # Handle 3D diagonal movements
        # XY diagonal (z=0)
        if motion.x and motion.y and not motion.z:
            # if exists jump point at horizontal or vertical
            x_dir = Node((motion.x, 0, 0), None, 1, None)
            y_dir = Node((0, motion.y, 0), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, y_dir):
                return new_node

        # XZ diagonal (y=0)
        elif motion.x and not motion.y and motion.z:
            x_dir = Node((motion.x, 0, 0), None, 1, None)
            z_dir = Node((0, 0, motion.z), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, z_dir):
                return new_node

        # YZ diagonal (x=0)
        elif not motion.x and motion.y and motion.z:
            y_dir = Node((0, motion.y, 0), None, 1, None)
            z_dir = Node((0, 0, motion.z), None, 1, None)
            if self.jump(new_node, y_dir) or self.jump(new_node, z_dir):
                return new_node

        # 3D diagonal (all three axes)
        elif motion.x and motion.y and motion.z:
            # Check all three 2D diagonal planes
            xy_dir = Node((motion.x, motion.y, 0), None, 1, None)
            xz_dir = Node((motion.x, 0, motion.z), None, 1, None)
            yz_dir = Node((0, motion.y, motion.z), None, 1, None)
            if (self.jump(new_node, xy_dir) or
                self.jump(new_node, xz_dir) or
                self.jump(new_node, yz_dir)):
                return new_node

        # if exists forced neighbor
        if self.detectForceNeighbor(new_node, motion):
            return new_node
        else:
            return self.jump(new_node, motion)

    def detectForceNeighbor(self, node, motion):
        """
        Detect forced neighbor of node in 3D space.

        Parameters:
            node (Node): current node
            motion (Node): the motion that current node executes

        Returns:
            flag (bool): True if current node has forced neighbor else False
        """
        x, y, z = node.current
        x_dir, y_dir, z_dir = motion.current

        # Axial movements (along single axis)
        # X-axis movement
        if x_dir and not y_dir and not z_dir:
            # Check Y and Z perpendicular directions
            for dy in [-1, 1]:
                if (x, y + dy, z) in self.obstacles and \
                    (x + x_dir, y + dy, z) not in self.obstacles:
                    return True
            for dz in [-1, 1]:
                if (x, y, z + dz) in self.obstacles and \
                    (x + x_dir, y, z + dz) not in self.obstacles:
                    return True

        # Y-axis movement
        elif not x_dir and y_dir and not z_dir:
            # Check X and Z perpendicular directions
            for dx in [-1, 1]:
                if (x + dx, y, z) in self.obstacles and \
                    (x + dx, y + y_dir, z) not in self.obstacles:
                    return True
            for dz in [-1, 1]:
                if (x, y, z + dz) in self.obstacles and \
                    (x, y + y_dir, z + dz) not in self.obstacles:
                    return True

        # Z-axis movement
        elif not x_dir and not y_dir and z_dir:
            # Check X and Y perpendicular directions
            for dx in [-1, 1]:
                if (x + dx, y, z) in self.obstacles and \
                    (x + dx, y, z + z_dir) not in self.obstacles:
                    return True
            for dy in [-1, 1]:
                if (x, y + dy, z) in self.obstacles and \
                    (x, y + dy, z + z_dir) not in self.obstacles:
                    return True

        # Diagonal movements in 2D planes
        # XY diagonal (z constant)
        elif x_dir and y_dir and not z_dir:
            if (x - x_dir, y, z) in self.obstacles and \
                (x - x_dir, y + y_dir, z) not in self.obstacles:
                return True
            if (x, y - y_dir, z) in self.obstacles and \
                (x + x_dir, y - y_dir, z) not in self.obstacles:
                return True

        # XZ diagonal (y constant)
        elif x_dir and not y_dir and z_dir:
            if (x - x_dir, y, z) in self.obstacles and \
                (x - x_dir, y, z + z_dir) not in self.obstacles:
                return True
            if (x, y, z - z_dir) in self.obstacles and \
                (x + x_dir, y, z - z_dir) not in self.obstacles:
                return True

        # YZ diagonal (x constant)
        elif not x_dir and y_dir and z_dir:
            if (x, y - y_dir, z) in self.obstacles and \
                (x, y - y_dir, z + z_dir) not in self.obstacles:
                return True
            if (x, y, z - z_dir) in self.obstacles and \
                (x, y + y_dir, z - z_dir) not in self.obstacles:
                return True

        # 3D diagonal (all three axes)
        elif x_dir and y_dir and z_dir:
            # Check forced neighbors in each plane
            if (x - x_dir, y, z) in self.obstacles and \
                (x - x_dir, y + y_dir, z + z_dir) not in self.obstacles:
                return True
            if (x, y - y_dir, z) in self.obstacles and \
                (x + x_dir, y - y_dir, z + z_dir) not in self.obstacles:
                return True
            if (x, y, z - z_dir) in self.obstacles and \
                (x + x_dir, y + y_dir, z - z_dir) not in self.obstacles:
                return True

        return False

