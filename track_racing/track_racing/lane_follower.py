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
        
        # Topics
        self.drive_topic = "/drive" if self.simulation else "/vesc/input/navigation"

        # Configuration
        
        
	    # Publishers and subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.lane_sub = self.create_subscription(PoseArray, "/trajectory/current", self.lane_cb, 1)

        # Callback functions here
        # self.timer = self.create_timer(1 / 20, self.on_tick)

        # State
        # self.distance_pid = PID(self, self.P_0, self.I_0, self.D_0)
        # self.slope_pid = PID(self, self.P_1, self.I_1, self.D_1)

    @staticmethod
    def parse_pid(arg: str):
        return (float(x.strip()) for x in arg[1:-1].split(","))
    
    def log(self, s):
        """Short-hand for logging messages"""
        self.get_logger().info(str(s))

    def on_tick(self):
        self.get_logger().info("Wall follower is running...")

        msg = AckermannDriveStamped()

        # Blend distance and slope controllers
        if self.distance_pid.control is None:
            return
        control = self.distance_pid.control + self.slope_pid.control

        msg.header.frame_id = self.BASE_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.VELOCITY * max(1 - 0.02 * control ** 4, 0.5)
        msg.drive.steering_angle = -control

        # Stop moving for 0.5s when a teleport is detected (i.e. test cases)
        if (self.get_clock().now().nanoseconds - self.cooldown) * 1e-9 < 0.5:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0

        self.drive_publisher.publish(msg)

    def lane_cb(self, msg: PoseArray):
        # Least-squares
        x = np.array([p.position.x for p in msg.poses])
        y = np.array([p.position.y for p in msg.poses])
        n = len(x)
        if n == 0:
            return
        m = (n * sum(x * y) - sum(x) * sum(y)) / (n * sum(x ** 2) - sum(x) ** 2)
        b = (sum(y) - m * sum(x)) / n

        if np.isnan(m) or np.isnan(b):
            return

        self.log((m, b))
        # VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")
        
        # self.distance_pid(self.SIDE * self.DESIRED_DISTANCE - b)
        # self.slope_pid(-m)


def main():
    rclpy.init()
    try:
        rclpy.spin(LaneFollower())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
