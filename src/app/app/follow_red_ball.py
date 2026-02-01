#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import math
import time
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import sdk.pid as pid
import sdk.common as common

class FollowRedBallNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        
        # Parameters
        self.target_radius = 40 # Target pixel radius for distance
        
        self.image_width = 640
        self.image_height = 480
        
        # PID Controllers
        self.pid_yaw = pid.PID(0.005, 0.0, 0.001) 
        self.pid_dist = pid.PID(0.005, 0.0, 0.001)

        self.machine_type = os.environ.get('MACHINE_TYPE', 'ROSOrin_Mecanum')

        # Communication
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # Joystick Override
        self.joy_sub = self.create_subscription(Twist, '/joy_teleop/cmd_vel', self.joy_callback, 1)
        self.last_joy_time = 0
        self.joy_active_duration = 2.0 

        # Subscribers
        if not self.machine_type or 'ROSOrin' in self.machine_type: 
            self.camera_topic = '/depth_cam/rgb0/image_raw'
        else:
            self.camera_topic = '/usb_cam/image_raw'
            
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 1)

        self.image_pub = self.create_publisher(Image, '/follow_red_ball/result_image', 1)
        self.get_logger().info('Follow Red Ball Node Started')

    def joy_callback(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_joy_time = time.time()

    def image_callback(self, msg):
        # Joystick Override Check
        if time.time() - self.last_joy_time < self.joy_active_duration:
             pass 
             # We still want to see the video even if override is active, so don't return early.
             # Just disable tracking commands later.

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = cv_image.shape[:2]

            # Blur to reduce noise
            blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # Use High Saturation to exclude wood/skin
            lower_red1 = np.array([0, 150, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 150, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            tracking_active = True
            if time.time() - self.last_joy_time < self.joy_active_duration:
                tracking_active = False
                cv2.putText(cv_image, "JOYSTICK OVERRIDE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            valid_contour_found = False
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Check circularity
                perimeter = cv2.arcLength(largest_contour, True)
                area = cv2.contourArea(largest_contour)
                if perimeter == 0:
                    circularity = 0
                else:
                    circularity = 4 * math.pi * area / (perimeter * perimeter)

                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                
                # Filter by size AND circularity (Ball should be > 0.6)
                if radius > 15 and circularity > 0.5:
                    valid_contour_found = True
                    # Draw visual feedback
                    center = (int(x), int(y))
                    cv2.circle(cv_image, center, int(radius), (0, 255, 0), 2)
                    cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
                    
                    text = f"R:{int(radius)} Circ:{circularity:.2f}"
                    cv2.putText(cv_image, text, (int(x)+10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    if tracking_active:
                        self.track_ball(x, radius)
            
            if not valid_contour_found or not contours:
                self.stop_robot()
            
            # Publish debug image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def track_ball(self, cx, radius):
        # Yaw control
        error_x = self.image_width / 2 - cx
        self.pid_yaw.update(error_x)
        
        # Distance control
        error_dist = self.target_radius - radius
        self.pid_dist.update(error_dist)
        
        twist = Twist()
        
        # Yaw
        if abs(error_x) > 20:
             # Inverted logic: user reported robot turns away.
             twist.angular.z = -1.0 * common.set_range(self.pid_yaw.output, -1.0, 1.0)
        else:
             self.pid_yaw.clear()
             
        # Linear (Move forward if ball is small/far, backward if large/close)
        # Note: radius is inversely proportional to distance
        if abs(error_dist) > 10:
            # PID output is positive when error is positive (ball too small/far)
            # We want to move FORWARD (positive X) in this case.
            # If the user reported the robot backing up when far, then the robot's X axis might be inverted relative to expectations
            # OR the PID output sign needs flipping.
            # User said: "Backs up when ball gets further away" (Radius small -> Error Positive -> Output Positive -> Backs up?)
            # If Positive Output = Back up, then we need to negate it to move forward.
            twist.linear.x = -1.0 * common.set_range(self.pid_dist.output, -0.3, 0.3)
        else:
            self.pid_dist.clear()
            
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.pid_yaw.clear()
        self.pid_dist.clear()

def main():
    node = FollowRedBallNode('follow_red_ball')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
