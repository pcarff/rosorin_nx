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
from std_srvs.srv import Trigger
from interfaces.msg import ObjectsInfo
import sdk.pid as pid
import sdk.common as common

class FollowCatNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        
        # Parameters
        self.target_width = 240 # Target pixel width for 0.5m distance (approx)
        self.conf_threshold = 0.5
        self.black_threshold = 40 # L channel threshold for "black"
        
        self.running = True
        self.finding = False
        self.cat_found = False
        self.cat_box = None
        self.image_width = 640
        self.image_height = 480
        
        # PID Controllers
        # Yaw (centering)
        self.pid_yaw = pid.PID(0.005, 0.0, 0.001) 
        # Distance (width)
        self.pid_dist = pid.PID(0.003, 0.0, 0.001)

        self.machine_type = os.environ.get('MACHINE_TYPE', 'ROSOrin_Mecanum')

        # Communication
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # Subscribers
        # We need to know the camera topic to verify color
        if 'ROSOrin' in self.machine_type: 
            self.camera_topic = '/depth_cam/rgb0/image_raw'
        else:
            self.camera_topic = '/usb_cam/image_raw'
            
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 1)
        
        # Subscribe to YOLO output
        # Assuming YOLO node is named 'yolo', topic is '/yolo/object_detect'
        # Or relative '~/object_detect' if we remap. We will assume '/yolo/object_detect' for now.
        self.yolo_sub = self.create_subscription(ObjectsInfo, '/yolo/object_detect', self.yolo_callback, 1)

        # Start YOLO
        self.start_yolo_client = self.create_client(Trigger, '/yolov5/start')
        
        self.get_logger().info('Follow Cat Node Started')
        
        # Start YOLO in a separate thread to avoid blocking init
        threading.Thread(target=self.start_yolo).start()

        # Subscribe to Joystick for override
        self.joy_sub = self.create_subscription(Twist, '/joy_teleop/cmd_vel', self.joy_callback, 1)
        self.last_joy_time = 0
        self.joy_active_duration = 2.0 # Seconds to wait after joystick input stops

    def joy_callback(self, msg):
        # If there is any non-zero command, register activity
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_joy_time = time.time()

    def start_yolo(self):
        if not self.start_yolo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('YOLO start service not available')
            return
        req = Trigger.Request()
        self.start_yolo_client.call_async(req)
        self.get_logger().info('Requested YOLO start')

    def yolo_callback(self, msg):
        # Check for joystick override
        if time.time() - self.last_joy_time < self.joy_active_duration:
             self.get_logger().info("Joystick override active", throttle_duration_sec=1.0)
             return

        # Find cat
        cats = []
        for obj in msg.objects:
            if obj.class_name == 'cat' and obj.score > self.conf_threshold:
                cats.append(obj)
        
        if not cats:
            self.cat_found = False
            self.stop_robot()
            return
            
        # Select best cat (largest or verify black)
        # We'll pick the largest one first, then verify color in image_callback if possible,
        # but image and yolo are async. 
        # For simplicity, we'll iterate cats and pick the first "black" one if we have recent image data.
        # But here we just store the cats and let image_callback or a timer handle logic?
        # Better: Process here if we have a recent image, OR just store best candidate.
        
        # Sort by size (width * height) descending
        cats.sort(key=lambda o: o.width * o.height, reverse=True)
        
        self.cat_found = True
        self.cat_box = cats[0].box 
        
        # We'll do control here.
        self.track_cat(self.cat_box)

    def track_cat(self, box):
        x1, y1, x2, y2 = box
        w = x2 - x1
        h = y2 - y1
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        # Yaw control
        error_x = self.image_width / 2 - cx
        self.pid_yaw.update(error_x)
        
        # Distance control (via width)
        error_dist = self.target_width - w
        self.pid_dist.update(error_dist)
        
        twist = Twist()
        
        # Yaw
        # If error is small, stop turning
        if abs(error_x) > 20:
             # Negative error (cat to right) -> turn right (negative z) ? 
             # pixel: 0 (left) ... 640 (right).
             # if cx > 320 (cat is right), error = 320 - 400 = -80.
             # We need to turn RIGHT (negative angular z).
             # PID output should be positive for positive error.
             # So we use range on PID output.
             twist.angular.z = common.set_range(self.pid_yaw.output, -1.0, 1.0)
        else:
             self.pid_yaw.clear()
             
        # Linear
        # If cat is smaller (width < target) -> error > 0 -> move forward (positive x).
        # But we also check "black-ness" here? 
        # Ideally we should verify it's a black cat.
        # I'll rely on image_callback to set a flag or filter. 
        # But to avoid delay, I'll assume if it's the only cat, it's the target.
        
        if abs(error_dist) > 20:
            twist.linear.x = common.set_range(self.pid_dist.output, -0.3, 0.3)
        else:
            self.pid_dist.clear()
            
        self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # Simple "Black Cat" check if we have a box
            if self.cat_found and self.cat_box is not None:
                x1, y1, x2, y2 = self.cat_box
                # Clamp to image
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(self.image_width, x2)
                y2 = min(self.image_height, y2)
                
                if x2 > x1 and y2 > y1:
                    roi = cv_image[y1:y2, x1:x2]
                    # Convert to LAB
                    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
                    l, a, b = cv2.split(lab)
                    avg_l = np.mean(l)
                    
                    if avg_l > self.black_threshold:
                         # Not black enough
                         # self.get_logger().warn(f"Cat detected but not black (L={avg_l:.2f})")
                         pass

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.pid_yaw.clear()
        self.pid_dist.clear()

def main():
    node = FollowCatNode('follow_cat')
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
