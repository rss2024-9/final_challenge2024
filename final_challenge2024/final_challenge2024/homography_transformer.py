#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from stop_msgs.msg import PixelLocation, PhysicalLocation
from geometry_msgs.msg import Point

from math import sqrt

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[354, 212],
                   [242, 217],
                   [354, 277],
                   [565, 224]] # dummy points
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[36, 0],
                    [37, 18],
                    [15, 2],
                    [30, -18]] # dummy points
######################################################

METERS_PER_INCH = 0.0254


class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")

        self.stopsign_pub = self.create_publisher(PhysicalLocation, "/relative_stopsign", 10)
        # self.sign_marker_pub = self.create_publisher(Marker, "/stopsign_marker", 1)

        # self.stoplight_pub = self.create_publisher(PhysicalLocation, "/relative_stoplight", 10)
        # self.light_marker_pub = self.create_publisher(Marker, "/stoplight_marker", 1)


        #will this just simulate an object where the car will stop at in sim?
        self.marker_pub = self.create_publisher(Marker, "/stop_marker", 1)


        self.stopsign_px_sub = self.create_subscription(PixelLocation, "/relative_sign_px", self.sign_detection_callback, 1)
        self.stoplight_px_sub = self.create_subscription(PixelLocation, "/relative_light_px", self.light_detection_callback, 1)
        self.mouse_px_sub = self.create_subscription(Point, "/zed/rgb/image_rect_color_mouse_left", self.mouse_click_callback, 1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rclpy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.get_logger().info("Homography Transformer Initialized")

    def sign_detection_callback(self, msg):
        #Extract information from message
        u, v = msg.u, msg.v

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = PhysicalLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y

        # self.get_logger().info(f"Distance to cone @ ({x}, {y}): {sqrt(x**2 + y ** 2)}")

        self.draw_marker(x, y, "map")
        self.stopsign_pub.publish(relative_xy_msg)

    def light_detection_callback(self, msg):
            #TODO: make this actually be for stoplight
            #Extract information from message
            u, v = msg.u, msg.v

            #Call to main function
            x, y = self.transformUvToXy(u, v)

            #Publish relative xy position of object in real world
            relative_xy_msg = PhysicalLocation()
            relative_xy_msg.x_pos = x
            relative_xy_msg.y_pos = y

            # self.get_logger().info(f"Distance to cone @ ({x}, {y}): {sqrt(x**2 + y ** 2)}")

            self.draw_marker(x, y, "map")
            self.stoplight_pub.publish(relative_xy_msg)
    
    def mouse_click_callback(self, msg: Point):
        u, v = msg.x, msg.y
        x, y = self.transformUvToXy(u, v)

        self.get_logger().info(f"({x}m, {y}m)")

        self.draw_marker(x, y, "map")


    def transformUvToXy(self, u, v):
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
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, x, y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        #todo make this for both stopsigns and stoplights
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()