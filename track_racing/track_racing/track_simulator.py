import numpy as np
import cv2 as cv
import rclpy
import os
import trimesh

from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from pyrender import OffscreenRenderer, Scene, PerspectiveCamera, PointLight, RenderFlags, Mesh
from trimesh.primitives import Sphere
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from math import pi

os.environ["PYOPENGL_PLATFORM"] = "egl"

class TrackSimulator(Node):
    def __init__(self):
        super().__init__("track_simulator")

        # Simulate a camera going around the track
        self.image_pub = self.create_publisher(Image, "/track_camera", 10)
        self.timer = self.create_timer(1 / 15, self.timer_cb)
        self.bridge = CvBridge()

        # The camera is a 3D renderer
        self.mesh = trimesh.load("/home/racecar/racecar_ws/src/final_challenge2024/track_racing/map/track.glb")
        self.scene = Scene.from_trimesh_scene(self.mesh)
        self.camera = self.scene.add(PerspectiveCamera(yfov=72 * (pi/180), znear=1e-4), pose=np.eye(4))
        self.light = self.scene.add(PointLight(color=np.ones(3), intensity=200.0), pose=np.eye(4))
        self.renderer = OffscreenRenderer(1080, 720)

        self.mouse_px_sub = self.create_subscription(Point, "/track_camera_mouse_left", self.mouse_click_callback, 1)

        p = np.eye(4)
        p[:3, 3] = [-3.0, 1.0, 0.0]
        self.scene.add(Mesh.from_trimesh(Sphere(0.2)), pose=p)

        # Get the car's actual position
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 1)

        self.log("Track simulator started!")
    
    def log(self, s):
        self.get_logger().info(str(s))

    def mouse_click_callback(self, msg: Point):
        u, v = msg.x, msg.y

        self.get_logger().info(f"({u}px, {v}px)")

    def timer_cb(self):
        img, _ = self.renderer.render(self.scene, RenderFlags.FLAT)
        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")

        self.image_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        r = R.from_quat(q) * R.from_rotvec([0, 0, np.pi / 2]) * R.from_rotvec([87.5 * np.pi / 180, 0, 0])

        pose = np.eye(4)
        pose[:3, :3] = r.as_matrix()
        pose[:3, 3] = [-x, -y, 0.3]

        self.scene.set_pose(self.camera, pose)
        self.scene.set_pose(self.light, pose)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(TrackSimulator())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()