import rclpy
import numpy as np
import scipy.ndimage as scipy
import itertools
import cv2 as cv
import json

from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path, Odometry

from skimage.transform import rescale
from astar import AStar
from math import isnan

DOWNSAMPLE = 8
STATA_SCALING = 0.0504
STATA_ORIGIN = [25.900000, 48.50000]
STATA_PATH = "/home/racecar/racecar_ws/src/final_challenge2024/vehicular_manslaughter/map/stata_basement.png"
CENTER_LINE_PATH = "/home/racecar/racecar_ws/src/final_challenge2024/vehicular_manslaughter/map/full-lane.traj"
N_POINTS = 1

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        # Load & process data
        ## Occupancy Grid
        self.stata_basement = cv.imread(STATA_PATH, cv.IMREAD_COLOR)
        self.map = create_occupancy_grid(self.stata_basement)
        self.astar = OccupancyGridPathPlanner(self.map)
        
        ## Center points (in pixel coordinates)
        with open(CENTER_LINE_PATH, "r") as f:
            # Parse .traj file
            cline = [(p["x"], p["y"]) for p in json.load(f)["points"]]

            # Make into pixel coordinates
            self.cline = subdivide_path(self.irl_to_pixel(cline), 4).astype(int)
        
        # State
        self.start = np.zeros(2)
        self.points = []

        # Subscriptions
        self.create_subscription(PointStamped, "/clicked_point", self.click_cb, 1)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 1)

        # Publishers
        self.cline_viz = self.create_publisher(Path, "center_line", 1)
        self.path_viz = self.create_publisher(Path, "computed_path", 1)
        self.path_pub = self.create_publisher(PoseArray, "/trajectory/current", 1)

        self.cline_viz.publish(self.create_path_msg(self.cline))

        # OK!
        self.log("Path planner initialized!")
    
    def log(self, s):
        """Short-hand for logging messages"""
        self.get_logger().info(str(s))

    def path_plan(self):
        """
        Path plan, this method expects and returns pixel coordinates (y, x)
        """
        points = [self.start, *self.points]
        path = []

        for (k, (start, goal)) in enumerate(zip(points, points[1:])):
            # Compute the path endpoints along the center line
            if k == 0:
                i, closest_start = closest_point_on_path((start[1], start[0]), self.cline)
            else:
                # Otherwise recyle last one
                i, closest_start = j, closest_end
            j, closest_end = closest_point_on_path((goal[1], goal[0]), self.cline)

            # Compute the path along the center line
            cline_path = self.cline[i+1:j+1] if j > i else self.cline[j+1:i+1][::-1]
            # Offset to drive on the right side of the road
            cline_path = offset_path(cline_path, -5)
            # Convert to (y, x) for consistency with a-star
            cline_path = [(y, x) for (x, y) in cline_path]

            # Compute the path from start point to center line, and center line to endpoint
            if k == 0:
                start_path = self.astar.astar(start, closest_start.astype(int)[::-1])
            else:
                start_path = end_path[::-1]
            end_path = self.astar.astar(closest_end.astype(int)[::-1], goal)

            if start_path is None:
                print("No path found from start point to center line!")
                return
            if end_path is None:
                print("No path found from center line to end point!")
                return

            # Go in reverse on start paths, forward on other paths
            start_path_z = [(y, x, 0 if k == 0 else 1) for (y, x) in start_path]
            cline_path_z = [(y, x, 0) for (y, x) in cline_path]
            end_path_z = [(y, x, 0) for (y, x) in end_path]

            path += start_path_z + cline_path_z + end_path_z
        return np.array(path)

    def click_cb(self, msg: PointStamped):
        x, y = self.irl_to_pixel([msg.point.x, msg.point.y])
        
        self.points.append((y, x))

        if len(self.points) >= N_POINTS:
            # Compute the path in pixels (y, x) -> (x, y)
            path = self.path_plan()
            if path is None:
                return
            path = path[:, [1, 0, 2]]

            self.log("Path planned!")

            # Publish
            self.path_viz.publish(self.create_path_msg(path))
            self.path_pub.publish(self.create_pose_array_msg(path))

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        if isnan(pos.x) or isnan(pos.y):
            return
        x, y = self.irl_to_pixel([pos.x, pos.y])

        if sum((self.start - np.array([y, x])) ** 2) > 2:
            self.points = []
            self.log("Reset the path!")
        
        self.start = (y, x)

    def pixel_to_irl(self, p):
        p = np.copy(p).astype(float)
        if len(p.shape) == 2:
            p[:, 0] *= -1
        else:
            p[0] *= -1
        p -= np.array([0, self.stata_basement.shape[0]])
        p += np.array(STATA_ORIGIN) / STATA_SCALING
        p *= STATA_SCALING

        return p

    def irl_to_pixel(self, p):
        p = np.array(p) / STATA_SCALING
        p -= np.array(STATA_ORIGIN) / STATA_SCALING
        p += np.array([0, self.stata_basement.shape[0]])
        if len(p.shape) == 2:
            p[:, 0] *= -1
        else:
            p[0] *= -1

        return p.astype(int)
    
    def create_path_msg(self, points):
        """
        Expects pixel coordinates (x, y)
        """
        msg = Path()

        msg.header.frame_id = "/map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y, *_) in points:
            x, y = self.pixel_to_irl((x, y))
            
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)
        return msg
    
    def create_pose_array_msg(self, points):
        """
        Expects pixel coordinates (x, y)
        """
        msg = PoseArray()

        msg.header.frame_id = "/map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y, z) in points:
            x, y = self.pixel_to_irl((x, y))
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0

            msg.poses.append(pose)
        return msg


class OccupancyGridPathPlanner(AStar):
    def __init__(self, map):
        self.map = rescale(map, 1/DOWNSAMPLE)

    def neighbors(self, node):
        # Node is the pixel location
        y, x = node
        for (dx, dy) in itertools.product([-1, 0, 1], repeat=2):
            if dx == 0 and dy == 0:
                continue
            if not (0 <= x + dx < self.map.shape[1]):
                continue
            if not (0 <= y + dy < self.map.shape[0]):
                continue
            if not self.map[y + dy][x + dx]:
                continue
            yield (y + dy, x + dx)
    
    def distance_between(self, n1, n2):
        return ((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2) ** 0.5
    
    def heuristic_cost_estimate(self, current, goal):
        return self.distance_between(current, goal)
    
    def astar(self, start, goal):
        start = tuple(n // DOWNSAMPLE for n in start)
        goal = tuple(n // DOWNSAMPLE for n in goal)
        
        path = super().astar(start, goal)
        if path:
            return [(y * DOWNSAMPLE, x * DOWNSAMPLE) for (y, x) in path]
        return None


# Adapted from:
# https://math.stackexchange.com/a/3128850
def closest_point_on_line(p, a, b):
    def vector_to_segment(t, p, a, b):
        return (1 - t) * a + t * (b - p)
    
    v = b - a
    u = a - p
    vu = np.dot(v, u)
    vv = np.dot(v, v)
    t = -vu / vv
    if 0 <= t <= 1:
        return vector_to_segment(t, np.zeros(2), a, b)
    
    g0 = sum(vector_to_segment(0, p, a, b) ** 2)
    g1 = sum(vector_to_segment(1, p, a, b) ** 2)
    
    return a if g0 <= g1 else b

def closest_point_on_path(p, path):
    best_p = None
    best_d = float("inf")
    best_i = -1
    for (i, (p0, p1)) in enumerate(zip(path, path[1:])):
        this_p = closest_point_on_line(p, p0, p1)
        this_d = sum((p - this_p) ** 2)
        if this_d < best_d:
            best_p = this_p
            best_d = this_d
            best_i = i
    return best_i, best_p

def subdivide_path(path, n):
    if n == 1:
        return np.array(path)
    out = []
    for (p0, p1) in zip(path, path[1:]):
        out.append(p0)
        out.append((p0 + p1) / 2)
    out.append(path[-1])

    return subdivide_path(out, n - 1)

def offset_path(path, dist):
    out = []
    o = np.zeros(2)
    for (p0, p1) in zip(path, path[1:]):
        d = p1 - p0
        lo = o
        o = np.array([d[1], -d[0]]) / np.linalg.norm(d)
        out.append(p0 + (o + lo) * dist)
    out.append(path[-1] + o * dist)

    return out

def create_occupancy_grid(img):
    img = np.average(img, axis=2)                   # Convert to grey-scale
    img = np.greater(img, 0.9)                      # White pixels represent holes in occupancy grid
    img = scipy.binary_erosion(img, iterations=10)  # Erosion adds a safe distance from walls, removes bumps

    return img


def main():
    rclpy.init()
    try:
        rclpy.spin(PathPlanner())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()