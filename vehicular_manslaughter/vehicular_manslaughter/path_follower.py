import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray

from vehicular_manslaughter.pid import PID

from tf_transformations import euler_from_quaternion
from math import cos, sin, acos

class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")

        # State
        self.trajectory = np.empty((0, 3))
        self.index = 0
        self.position = np.zeros(2)
        self.heading = np.array([1, 0])
        # self.slope_pid = PID(self, 0.3, 0.0, 0.4)
        # self.distance_pid = PID(self, 0.15, 0.0, 0.0)
        self.pid = PID(self, 0.5, 0.0, 0.0)

        # Subscriptions
        self.create_subscription(PoseArray,"/trajectory/current", self.trajectory_cb, 1)
        self.create_subscription(Odometry,"/odom", self.odom_cb, 1)
        self.create_timer(1 / 20, self.tick_cb)
        
        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)

    def log(self, s):
        """Short-hand for logging messages"""
        self.get_logger().info(str(s))

    def drive(self, speed, steer):
        """Short-hand for sending ackermann driving"""
        msg = AckermannDriveStamped()
        msg.header.frame_id = "/base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)

        self.drive_pub.publish(msg)

    def trajectory_cb(self, msg: PoseArray):
        self.log("Got a new trajectory!")
        
        self.trajectory = np.array([(pose.position.x, pose.position.y, pose.position.z) for pose in msg.poses])
        self.index = 0

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        self.position = np.array([p.x, p.y])
        self.heading = np.array([cos(t), sin(t)])

    def tick_cb(self):
        # End of trajectory
        if self.index >= self.trajectory.shape[0] - 1:
            self.drive(0, 0)
            return
        
        heading = self.trajectory[self.index + 1][:2] - self.trajectory[self.index][:2]
        heading /= np.linalg.norm(heading)

        # heading = self.trajectory[self.index][:2] - self.position
        dist = np.linalg.norm(self.trajectory[self.index][:2] - self.position)

        # Next point on path
        if dist < 0.5:
            self.index += 1

        self.slope_pid(-acos(np.dot(heading, self.heading)))
        self.distance_pid(-dist)

        self.log((acos(np.dot(heading, self.heading)), self.index))

        if self.slope_pid.control is None:
            return
        self.drive(1.0, self.slope_pid.control + self.distance_pid.control)

        
def main():
    rclpy.init()
    try:
        rclpy.spin(PathFollower())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()