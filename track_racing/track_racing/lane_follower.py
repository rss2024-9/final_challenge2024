import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from math import pi

from track_racing.pid import PID


class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")

        self.simulation = self.declare_parameter("simulation", True).get_parameter_value().bool_value
        
        # Topics & Argments
        self.drive_topic = "/drive" if self.simulation else "/vesc/input/navigation"
        self.velocity = self.declare_parameter("velocity", 3.0).get_parameter_value().double_value
        self.distance_pid_params = self.declare_parameter("b_pid", "(1.0, 0.0, 0.0)").get_parameter_value().string_value
        self.slope_pid_params = self.declare_parameter("m_pid", "(1.0, 0.0, 0.0)").get_parameter_value().string_value
        self.max_steer = self.declare_parameter("max_steer", 15.0).get_parameter_value().double_value * pi / 180
        self.distance_pid_setpoint = 0.5
        self.slope_pid_setpoint = 0.3

        # PID
        self.distance_pid = PID(self, *LaneFollower.parse_pid(self.distance_pid_params))
        self.slope_pid = PID(self, *LaneFollower.parse_pid(self.slope_pid_params))

	    # Publishers and subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.lane_sub = self.create_subscription(PoseArray, "/trajectory/current", self.lane_cb, 1)

        self.log(f"Initialized the lane follower:")
        self.log(f"    Velocity: {self.velocity}")
        self.log(f"    Distance PID: {self.distance_pid_params}")
        self.log(f"    Slope PID: {self.slope_pid_params}")

    @staticmethod
    def parse_pid(arg: str):
        return (float(x.strip()) for x in arg[1:-1].split(","))
    
    def log(self, s):
        """Short-hand for logging messages"""
        self.get_logger().info(str(s))
    
    def lane_cb(self, msg: PoseArray):
        # Least-squares
        x = np.array([p.position.x for p in msg.poses])
        y = np.array([p.position.y for p in msg.poses])
        n = len(x)
        if n == 0:
            return
        d = (n * sum(x ** 2) - sum(x) ** 2)
        if d == 0:
            return
        m = (n * sum(x * y) - sum(x) * sum(y)) / d
        b = (sum(y) - m * sum(x)) / n

        if np.isnan(m) or np.isnan(b):
            return

        self.log((m, b))
        
        # These are determined experimentally by placing the car where it should be on the lane,
        # and taking note of the (m, b) observed above ^
        self.distance_pid(self.distance_pid_setpoint - b)
        self.slope_pid(self.slope_pid_setpoint - m)

        if self.distance_pid.control is None:
            return

        # Create the driving message
        out = AckermannDriveStamped()

        # Blend distance and slope controllers
        control = self.distance_pid.control + self.slope_pid.control

        out.header.frame_id = "/base_link"
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = self.velocity
        out.drive.steering_angle = min(self.max_steer, max(-self.max_steer, control))

        self.drive_pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(LaneFollower())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
