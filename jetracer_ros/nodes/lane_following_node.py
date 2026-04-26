#!/usr/bin/env python
# jetracer_ros/scripts/lane_following_node.py

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path

from bev import BEVTransform
from lane_detection import LaneDetector
from sliding_window import SlidingWindow
from waypoint_generation import WaypointGenerator
from control import LaneController

class LaneFollowingNode:
    def __init__(self):
        rospy.init_node('lane_following_node', anonymous=True)

        # Load parameters
        config = rospy.get_param('~lane_following', rospy.get_param('lane_following', {}))
        
        self.enabled = config.get('enabled', True)
        self.emergency_stop = False
        
        # Image ROI
        self.roi_y_min = config.get('image', {}).get('roi_y_min', 240)
        self.roi_y_max = config.get('image', {}).get('roi_y_max', 480)
        self.resize_w = config.get('image', {}).get('resize_width', 640)
        self.resize_h = config.get('image', {}).get('resize_height', 480)

        # Modules
        bev_cfg = config.get('bev', {})
        self.bev = BEVTransform(bev_cfg.get('src_points'), bev_cfg.get('dst_points'), bev_cfg.get('output_width', 640), bev_cfg.get('output_height', 480))
        self.detector = LaneDetector(config.get('threshold', {}))
        self.sliding_window = SlidingWindow(config.get('sliding_window', {}))
        self.wp_gen = WaypointGenerator(config.get('lane_model', {}))
        self.controller = LaneController(config.get('control', {}))

        self.bridge = CvBridge()

        # Subscriptions
        img_topic = config.get('image', {}).get('input_topic', '/camera/image_raw')
        self.sub_img = rospy.Subscriber(img_topic, Image, self.image_cb, queue_size=1)
        self.sub_estop = rospy.Subscriber('/safety/emergency_stop', Bool, self.estop_cb)

        # Publishers
        self.pub_cmd = rospy.Publisher(config.get('control', {}).get('output_topic', '/cmd_vel_lane'), Twist, queue_size=1)
        self.pub_wp = rospy.Publisher(config.get('waypoints', {}).get('output_topic', '/lane/waypoints'), Path, queue_size=1)
        
        self.pub_debug = config.get('debug', {}).get('publish_debug_image', True)
        if self.pub_debug:
            self.pub_debug_img = rospy.Publisher(config.get('debug', {}).get('debug_image_topic', '/lane/debug_image'), Image, queue_size=1)
            self.pub_bev_img = rospy.Publisher(config.get('debug', {}).get('bev_image_topic', '/lane/bev_image'), Image, queue_size=1)
        
        self.pub_status = rospy.Publisher(config.get('debug', {}).get('status_topic', '/lane/status'), String, queue_size=1)

        rospy.loginfo("Lane Following Node initialized.")

    def estop_cb(self, msg):
        self.emergency_stop = msg.data

    def image_cb(self, data):
        if not self.enabled or self.emergency_stop:
            self.publish_zero_cmd()
            self.pub_status.publish("DISABLED_OR_ESTOP")
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Preprocessing: Resize and ROI
        cv_img = cv2.resize(cv_img, (self.resize_w, self.resize_h))
        roi_img = cv_img[self.roi_y_min:self.roi_y_max, :]

        # 1. BEV Transform
        bev_img = self.bev.warp(roi_img)

        # 2. Thresholding
        binary_mask = self.detector.process(bev_img)

        # 3. Sliding Window
        left_fit, right_fit, sw_img, left_ok, right_ok = self.sliding_window.find_lanes(binary_mask)

        # 4. Waypoint Generation
        path, valid_path, center_points = self.wp_gen.generate(left_fit, right_fit, binary_mask.shape)
        
        if valid_path:
            self.pub_wp.publish(path)
            # 5. Control Command
            speed, steering = self.controller.compute_command(path)
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = steering
            self.pub_cmd.publish(cmd)
            self.pub_status.publish("ACTIVE")
        else:
            self.publish_zero_cmd()
            self.pub_status.publish("NO_LANE_DETECTED")

        # 6. Debug Publishing
        if self.pub_debug:
            # Draw center points on BEV image
            for pt in center_points:
                cv2.circle(sw_img, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), -1)
                
            try:
                self.pub_bev_img.publish(self.bridge.cv2_to_imgmsg(sw_img, "bgr8"))
                
                # Unwarp BEV overlay back to original image
                unwarped = self.bev.unwarp(sw_img)
                overlay = cv2.addWeighted(roi_img, 1.0, unwarped, 0.3, 0)
                final_img = cv_img.copy()
                final_img[self.roi_y_min:self.roi_y_max, :] = overlay
                self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(final_img, "bgr8"))
            except CvBridgeError as e:
                pass

    def publish_zero_cmd(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaneFollowingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
