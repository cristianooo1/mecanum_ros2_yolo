#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class AStarPlanner:
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def __init__(self, ox, oy, resolution, origin_x, origin_y, robot_radius=0.2):
        self.resolution = resolution
        self.min_x = origin_x
        self.min_y = origin_y
        self.max_x = origin_x + resolution * len(ox)
        self.max_y = origin_y + resolution * len(oy)
        self.rr = robot_radius
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    def planning(self, sx, sy, gx, gy):
        start = self.Node(self.calc_xy_index(sx, self.min_x),
                          self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal = self.Node(self.calc_xy_index(gx, self.min_x),
                         self.calc_xy_index(gy, self.min_y), 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start)] = start

        while open_set:
            c_id = min(open_set,
                       key=lambda i: open_set[i].cost + self.calc_heuristic(goal, open_set[i]))
            current = open_set[c_id]

            # Check goal
            if current.x == goal.x and current.y == goal.y:
                goal.parent_index = current.parent_index
                goal.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for dx, dy, dc in self.motion:
                node = self.Node(current.x + dx,
                                 current.y + dy,
                                 current.cost + dc,
                                 c_id)
                n_id = self.calc_index(node)
                if not self.verify(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        return self.build_path(goal, closed_set)

    def build_path(self, goal, closed):
        rx, ry = [], []
        n = goal
        while n.parent_index != -1:
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            n = closed[n.parent_index]
        rx.append(self.calc_position(n.x, self.min_x))
        ry.append(self.calc_position(n.y, self.min_y))
        return list(reversed(rx)), list(reversed(ry))

    def calc_position(self, idx, minp):
        return idx * self.resolution + minp + self.resolution / 2.0

    def calc_xy_index(self, pos, minp):
        return int((pos - minp) / self.resolution)

    def calc_index(self, node):
        return node.y * self.x_width + node.x

    @staticmethod
    def calc_heuristic(n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def verify(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)
        # out of bounds
        if px < self.min_x or py < self.min_y:
            return False
        if px >= self.max_x or py >= self.max_y:
            return False
        # obstacle
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        # grid dimensions
        self.x_width = int((self.max_x - self.min_x) / self.resolution)
        self.y_width = int((self.max_y - self.min_y) / self.resolution)

        # initialize free map
        self.obstacle_map = [[False] * self.y_width for _ in range(self.x_width)]

        # inflation radius in cells
        ir = int(math.ceil(self.rr / self.resolution))

        # mark and inflate each obstacle
        for obs_x, obs_y in zip(ox, oy):
            ix = self.calc_xy_index(obs_x, self.min_x)
            iy = self.calc_xy_index(obs_y, self.min_y)
            for dx in range(-ir, ir + 1):
                for dy in range(-ir, ir + 1):
                    x = ix + dx
                    y = iy + dy
                    if 0 <= x < self.x_width and 0 <= y < self.y_width:
                        cx = x * self.resolution + self.min_x + self.resolution / 2.0
                        cy = y * self.resolution + self.min_y + self.resolution / 2.0
                        if math.hypot(cx - obs_x, cy - obs_y) <= self.rr:
                            self.obstacle_map[x][y] = True

    @staticmethod
    def get_motion_model():
        # 8 connected grid
        return [
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
        ]


class AStarROS2Node(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map = None
        self.pose = None

        # subscriptions
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 1)
        self.sub_odom = self.create_subscription(
            Odometry, '/ddd/odom', self.odom_cb, 1)

        # target parameters
        self.declare_parameter('target_x', 0.8)
        self.declare_parameter('target_y', -0.1)
        self.target = (
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value
        )

        # publisher
        self.pub_path = self.create_publisher(Path, '/planned_path', 1)

    def map_cb(self, msg: OccupancyGrid):
        if self.map is None:
            self.map = msg
            self.try_plan()

    def odom_cb(self, msg: Odometry):
        if self.pose is None:
            self.pose = msg.pose.pose
            self.try_plan()

    def try_plan(self):
        if not (self.map and self.pose):
            return

        ox, oy = [], []
        info = self.map.info
        width, height = info.width, info.height
        res = info.resolution

        # extract obstacle world coords
        for idx, val in enumerate(self.map.data):
            if val > 50:
                x = idx % width
                y = idx // width
                wx = x * res + info.origin.position.x
                wy = y * res + info.origin.position.y
                ox.append(wx)
                oy.append(wy)

        # initialize planner with inflation
        planner = AStarPlanner(
            ox, oy, res,
            info.origin.position.x,
            info.origin.position.y,
            robot_radius=0.3  # adjust if needed
        )

        # plan
        sx = self.pose.position.x
        sy = self.pose.position.y
        gx, gy = self.target
        rx, ry = planner.planning(sx, sy, gx, gy)

        # publish Path
        path_msg = Path()
        path_msg.header = self.map.header
        for x, y in zip(rx, ry):
            ps = PoseStamped()
            ps.header = self.map.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.pub_path.publish(path_msg)

        # shutdown once done
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AStarROS2Node()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
