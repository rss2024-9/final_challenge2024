import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from stop_msgs.msg import PhysicalLocation
from geometry_msgs.msg import PointStamped

# from safety_controller_pkg.visualization_tools import VisualizationTools

#basically our safety controller code but for stopsigns and stoplights

class CityStoppingController(Node):
    def __init__(self):
        super().__init__("city_stopping_controller")
        hz = 20
        self.timer = self.create_timer(1 / hz, self.timer_callback)

        # Declare parameters to make them available for use
        self.declare_parameter("drive_topic", "/drive") 
        self.declare_parameter("stop_topic","/drive")
        self.declare_parameter("base_frame", "/base_link")

        #approx. locations of stoplights (classroom, hallway, vending machines) on map
        # self.stoplight_coords = [(10.4,16.6),(29.2,-34.1), (54.7,-22.9)]
        self.light_1_pose = (-10.8, 16.7)
        self.light_2_pose = (-31.2, 33.8)
        self.light_3_pose = (-55.1, 24.1)

        #set default euclid dists to all stoplights
        self.stoplight_1_dist = float('inf')
        self.stoplight_2_dist = float('inf')
        self.stoplight_3_dist = float('inf')


        #when stopping for stop sign, decrement this value
        #when this countdown is done, wait for cooldown to decrement
        #then reset both
        self.ignore_stopsigns = False
        self.stopsign_brake_time = 30 #can adjust i guess, plan to just decrement every time a drive command recieved
        self.stopsign_cooldown = 30
        
        # Fetch constants from the ROS parameter server
        # self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.STOP_TOPIC = self.get_parameter('stop_topic').get_parameter_value().string_value

        self.BASE_FRAME = self.get_parameter('base_frame').get_parameter_value().string_value
       
        
        # Publishers and subscribers
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, self.DRIVE_TOPIC, self.log_drive_command, 10)
        self.stopsign_subscriber = self.create_subscription(PhysicalLocation, '/relative_stopsign', self.on_stopsign, 10)
        self.detects_stoplight = self.create_subscription(Bool, '/detects_stoplight', self.on_stoplight, 10)
        self.ground_truth_subscriber = self.create_subscription(Odometry, '/pf/pose/odom', self.log_car_pose, 1)
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_callback, 1)

        self.stop_pub = self.create_publisher(AckermannDriveStamped, self.STOP_TOPIC, 1)
        self.error_pub = self.create_publisher(Float32,"city_stop_error",1)
        self.marker_pub = self.create_publisher(Marker, "/stop_light_marker", 1)
        
        #initiallize default last car pose to (0,0,0)
        self.last_car_pose = Odometry()
        self.last_car_pose.pose.pose.position.x = 0.0
        self.last_car_pose.pose.pose.position.y = 0.0
        self.last_car_pose.pose.pose.position.z = 0.0


        #max last_drive command at start to make safe initially
        self.last_drive_command = AckermannDriveStamped()
        self.last_drive_command.header.frame_id = self.BASE_FRAME
        self.last_drive_command.drive.speed = 0.0
        self.last_drive_command.drive.steering_angle = 0.0
        self.last_drive_command.header.stamp = self.get_clock().now().to_msg()

        #pre populate stop message
        self.stop_msg = AckermannDriveStamped() #pre populate stop message
        self.stop_msg.header.frame_id = self.BASE_FRAME
        self.stop_msg.drive.speed = 0.0
        self.stop_msg.drive.steering_angle = 0.0

        # markers = MarkerArray()
        for (id, (x, y)) in enumerate([self.light_1_pose, self.light_2_pose, self.light_3_pose, (1.0, 1.0)]):
            # markers.markers.append(self.draw_marker(x, y, id))
            self.draw_marker(x, y, id)
        # self.marker_pub.publish(markers)

        self.get_logger().info("started up city stopping node")

    def clicked_callback(self, msg):
        self.get_logger().info(f"{msg.point.x}, {msg.point.y}")

    def draw_marker(self, x, y, id):
        """
        Publish a marker to represent the stop light in rviz
        """
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.id = id
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        self.marker_pub.publish(marker)
        
    def timer_callback(self):
        #attempt to fix stuttering
        self.stop_pub.publish(self.last_drive_command) #this might cause issues

    def log_drive_command(self,msg):
        """
        Keeps internal record of last drive command issued
        msg:AckermannDriveStamped - drive command issued by wall follower
        return:None sets instance variable
        """
        
        self.last_drive_command = msg

        #decrement stopsign cooldown if currently ignoring stopsigns and cooldown time remaining
        if self.ignore_stopsigns and self.stopsign_cooldown:
            self.stopsign_cooldown-=1
        else: #stopsign cooldown complete
            #reset
            self.ignore_stopsigns = False
            self.stopsign_brake_time = 30
            self.stopsign_cooldown = 30

    def log_car_pose(self,msg):
        """
        Keeps internal record of ground truth pose of the car
        msg:Odometry - publishes Odometry message with msg.pose
        return:None sets instance variable
        """
        
        self.last_car_pose = msg
        x = self.last_car_pose.pose.pose.position.x
        y = self.last_car_pose.pose.pose.position.y

        #update distances to stoplights
        self.stoplight_1_dist = np.sqrt((x-self.light_1_pose[0])**2+(y-self.light_1_pose[1])**2)
        self.stoplight_2_dist = np.sqrt((x-self.light_2_pose[0])**2+(y-self.light_2_pose[1])**2)
        self.stoplight_3_dist = np.sqrt((x-self.light_3_pose[0])**2+(y-self.light_3_pose[1])**2)
    

    def on_stopsign(self,msg):
        """
        Monitors for stop signs and publishes stop command for 2 seconds before moving on

        #TODO: wuh oh just thought of a problem to debug later
        #TODO: also need to actually implement this
        (shit wait yeah it might just keep detecting that stop sign, maybe set an amount of time for stopsign to be ignored?
        but what if there's another stop sign right afterwards that we drive past?)

        msg:PhysicalLocation - relative physical location (x,y) of stop sign from car
        return:None - publishes directions to car if necessary
        """
        
        speed = self.last_drive_command.drive.speed
        t_reac = .05 #frequency msgs are sent at
        u = 1.4 #coefficient of friction used for cars in charts
        grav = 9.8 
        total_stop_d = speed*t_reac + (speed**2)/(2*u*grav)
        threshold = 1 #can adjust until stops consistently at 0.5-1m from light

        #these are relative to robot frame
        x = msg.x_pos
        y = msg.y_pos

        obj_dist = np.sqrt(x**2 + y**2)

        if obj_dist <= (total_stop_d + threshold) and self.stopsign_brake_time:
            #start braking
            error_msg = Float32()
            error_msg.data = abs(obj_dist-(total_stop_d+threshold))/(total_stop_d+threshold)
            self.error_pub.publish(error_msg)
            self.stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.stop_msg.drive.steering_angle = self.last_drive_command.drive.steering_angle
            
            self.get_logger().info("STOP! Breaking...")

            #TODO: find way to make it come to a full stop and then continue on without reacting to same stop sign
            self.stop_pub.publish(self.stop_msg)
            self.stopsign_brake_time -= 1 #stop for about one second (callback runs at 10Hz)
        elif obj_dist <= (total_stop_d + threshold) and not self.stopsign_brake_time and self.stopsign_cooldown: #within range and done braking, start cooldown
            self.ignore_stopsigns=True #start cooldown
            

    def on_stoplight(self,msg):
        """
        Monitors for red stoplights within 1m of car and publishes stop command until red stoplight no longer detected
        (relatively easier than stopsigns ngl)

        msg:PhysicalLocation - relative physical location (x,y) of red stoplight from car
        return:None - publishes directions to car if necessary
        """

        speed = self.last_drive_command.drive.speed
        t_reac = .05 #frequency msgs are sent at
        u = 1.4 #coefficient of friction used for cars in charts
        grav = 9.8 
        total_stop_d = speed*t_reac + (speed**2)/(2*u*grav)
        threshold = 1 #can adjust threshold until car consistently stops within 0.5-1m
        radius = total_stop_d + threshold

        detects_stoplight = msg.data

        if detects_stoplight and (self.light_1_pose <= radius or self.light_2_pose <= radius or self.light_3_pose <= radius):
            # error_msg = Float32()
            # error_msg.data = abs(obj_dist-(total_stop_d+threshold))/(total_stop_d+threshold)
            # self.error_pub.publish(error_msg)
            # above stuff was for experimental eval, might need to make later

            self.stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.stop_msg.drive.steering_angle = self.last_drive_command.drive.steering_angle
            self.stop_pub.publish(self.stop_msg)
            self.get_logger().info("LIGHT! Breaking...")


def main():
    rclpy.init()
    city_stopping_controller = CityStoppingController()
    try:
        rclpy.spin(city_stopping_controller)
    except KeyboardInterrupt:
        pass
    city_stopping_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
