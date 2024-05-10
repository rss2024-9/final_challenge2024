import numpy as np
import cv2 as cv
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path, Odometry
from cv_bridge import CvBridge
from skimage.morphology import binary_dilation, binary_erosion, skeletonize, square
from math import pi, cos, sin, atan2, isnan
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# Pixels
HOMOGRAPHY_IMAGE_PLANE = [
    [364, 175],
    [623, 170],
    [585, 217],
    [133, 222],
]
# Inches (+x forward, +y left)
HOMOGRAPHY_GROUND_PLANE = [
    [132.5, 3],
    [122, -97],
    [33.5, -25],
    [35, 35],
]
METERS_PER_INCH = 0.0254

# HOMOGRAPHY_SIMULATION_IMAGE_PLANE = [
#     [1210, 578],
# ]
# HOMOEGRAPHY_SIMULATION_GROUND_PLANE = [
#     []
# ]

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.simulation = self.declare_parameter("simulation", True).get_parameter_value().bool_value
        self.camera_topic = "/track_camera" if self.simulation else "/zed/zed_node/rgb/image_rect_color"
        
        # Subscription to the ZED2 camera
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, 5)
        self.bridge = CvBridge()

        # Publisher for debugging CV stuff
        self.debug_image_pub = self.create_publisher(Image, "/lane_debug_img", 10)

        # Homography maps 2D lanes to 3D
        self.homography, _ = cv.findHomography(
            np.array(HOMOGRAPHY_IMAGE_PLANE)[:, np.newaxis, :],
            np.array(HOMOGRAPHY_GROUND_PLANE)[:, np.newaxis, :] * METERS_PER_INCH,
        )
        self.lane_pub = self.create_publisher(PoseArray, "/trajectory/current", 1)

        # Using frames causes segfaults (ros is stupid and does not work) so we do it manually
        if self.simulation:
            self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 1)
            self.car = (np.array([0, 0]), R.identity())

        self.log("Lane detector initialized.")
    
    def log(self, s):
        """Short-hand for logging messages"""
        self.get_logger().info(str(s))
    
    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        viz, rho, theta = LaneDetector.find_lanes(img, sim=self.simulation)
        lines = LaneDetector.merge_lines(rho, theta)

        # Visualize
        for (rho, theta, _) in lines:
            m, b = LaneDetector.line_polar_to_slope_intercept(rho, theta)
            if m > -0.6:
                continue
            
            # y = mx + b --> x = (y - b) / m
            y = 200
            x = int((y - b) / m)

            cv.line(viz, *LaneDetector.line_polar_to_cartesian(rho, theta), (255, 0, 0), 2, cv.LINE_AA)
            cv.circle(viz, (x, y), 3, (0, 0, 255), -1)

            # Visualize but make it ~ 3D ~
            # msg2 = Path()

            # msg2.header.frame_id = "map"
            # msg2.header.stamp = self.get_clock().now().to_msg()
            # msg2.poses = []

            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()

            #for y in range(150, 200, 5):
            for y in np.array([150,200]):
                x = (y - b) / m
                x, y = self.homography_transform(x, y)

                if abs(x) > 10 or abs(y) > 10 or isnan(x) or isnan(y):
                    continue
                if self.simulation:
                    x, y, _ = self.car[1].apply(np.array([x, y, 0])) + self.car[0]

                # p = PoseStamped()
                # p.header.frame_id = "map"
                # p.header.stamp = msg2.header.stamp
                # p.pose.position.x = float(x)
                # p.pose.position.y = float(y)
                # p.pose.position.z = 0.0
                # msg2.poses.append(p)
                msg.poses.append(LaneDetector.pose_to_msg(x, y, 0.0))
            self.lane_pub.publish(msg)

            self.lane_pub.publish(msg)
        
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(viz, "bgr8"))
    
    def odom_cb(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        p = np.array([x, y, 0])
        r = R.from_quat(q)

        self.car = (p, r)

    @staticmethod
    def find_lanes(img, *, sim=False):
        """
        Computer-vision algorithm to detect lanes on a track. Uses polar representation.
        """
        # Down-sample image
        # img = cv.resize(img, (600, 400), interpolation=cv.INTER_NEAREST)

        viz = np.copy(img)

        # # Isolate the red track and lanes within it. This initially gets rid of the white lanes,
        # # but we get them back by dilating the mask horizontally.
        # hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # mask = cv.inRange(hsv, (0, 75, 100), (10, 200, 255) if sim else (10, 100, 180))
        # mask = binary_dilation(mask, rectangle(5, 150 if sim else 50))
        # img *= mask.astype(np.uint8)[:, :, np.newaxis]

        # Mask out the bright white lanes
        white = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img = np.greater(white, 150).astype(np.uint8) * 255

        # Everything near top of image is probably not relevant and tends to leak
        img[:100, :] = 0

        # Erosion removes any noise
        img = binary_erosion(img, square(4))
        # Dilation gives the lines their original thickness and fixes any discontinuities
        img = binary_dilation(img, square(3))

        # Do a copy here so that visualization sees every CV stuff above here but not below
        # viz = cv.cvtColor(np.copy(img).astype(np.uint8) * 255, cv.COLOR_GRAY2BGR)

        # Reduce lines to 1-pixel wide representation
        img = skeletonize(img)
        img = img.astype(np.uint8) * 127

        # Reduce look-ahead
        img[:200, :] = 0

        # Find the lines in polar representation
        lines = cv.HoughLines(img, 1, pi / 180, 50, None, 0, 0)
        if lines is None:
            return (viz, [], [])
        return (viz, lines[:, 0, 0], lines[:, 0, 1])

    @staticmethod
    def merge_lines(rho, theta, *, rho_threshold=50, theta_threshold=10):
        """
        Merges lines that are close together. Uses polar representation. Returned is a
        list of tuples for (rho, theta, count) where count is the number of lines that
        were merged into this one.
        """
        # Normalize the lines (positive rho)
        rho, theta = np.array(rho), np.array(theta)
        theta[rho < 0] += pi
        rho[rho < 0] *= -1

        out = []

        # Iterate each line in O(n^2)
        for (r0, t0) in zip(rho, theta):
            for (i, (r1, t1, count)) in enumerate(out):
                # Lines can be merged if they are nearby and of similar slope
                if abs(r0 - r1) > rho_threshold:
                    continue
                if abs(atan2(sin(t0 - t1), cos(t0 - t1))) > (theta_threshold * (pi / 180)):
                    continue

                # Merge at the average
                rho = (r1 * count + r0) / (count + 1)
                theta = atan2(sin(t1) * count + sin(t0), cos(t1) * count + cos(t0))
                out[i] = (rho, theta, count + 1)
                break
            else:
                # No line was found to merge, so it goes through as-is
                out.append((r0, t0, 1))
        
        return out
    
    def homography_transform(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        xy = np.dot(self.homography, np.array([[u], [v], [1]]))
        hxy = xy * (1.0 / xy[2, 0])

        x = hxy[0, 0]
        y = hxy[1, 0]

        return x, y
    
    @staticmethod
    def line_polar_to_cartesian(rho, theta, *, dist=10000):
        """
        Converts a line from polar to cartesian representation.
        """
        # Source:
        # https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        a = cos(theta)
        b = sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt0 = (int(x0 - (dist * b)), int(y0 + (a * dist)))
        pt1 = (int(x0 + (dist * b)), int(y0 - (a * dist)))

        return (pt0, pt1)
    
    @staticmethod
    def line_polar_to_slope_intercept(rho, theta):
        """
        Converts a line from polar to y = mx + b
        """
        m = (-cos(theta)/sin(theta)) if sin(theta) != 0 else float("inf")
        b = rho / sin(theta)

        return m, b
    
    @staticmethod
    def pose_to_msg(x, y, theta):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = 0.0
        
        quaternion = quaternion_from_euler(0.0, 0.0, theta)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        return msg


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(LaneDetector())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()