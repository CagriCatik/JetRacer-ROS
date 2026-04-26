import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy

class WaypointGenerator:
    def __init__(self, config):
        self.lane_width_m = config.get('lane_width_m', 0.35)
        self.xm_per_pix = config.get('xm_per_pix', 0.0015)
        self.ym_per_pix = config.get('ym_per_pix', 0.0030)
        self.count = config.get('count', 10)
        self.min_dist = config.get('min_distance_m', 0.2)
        self.max_dist = config.get('max_distance_m', 1.5)

    def generate(self, left_fit, right_fit, img_shape, frame_id="base_link"):
        """Generate waypoints along the centerline of the lane."""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        
        if left_fit is None and right_fit is None:
            return path, False

        # Generate Y points in BEV pixel coordinates (bottom of image is y=max, top is y=0)
        ploty = np.linspace(img_shape[0]-1, 0, self.count)
        
        center_fit_x = []
        for y in ploty:
            lx = left_fit[0]*y**2 + left_fit[1]*y + left_fit[2] if left_fit is not None else None
            rx = right_fit[0]*y**2 + right_fit[1]*y + right_fit[2] if right_fit is not None else None
            
            if lx is not None and rx is not None:
                cx = (lx + rx) / 2.0
            elif lx is not None:
                cx = lx + (self.lane_width_m / self.xm_per_pix) / 2.0
            elif rx is not None:
                cx = rx - (self.lane_width_m / self.xm_per_pix) / 2.0
            else:
                break
                
            # Convert to approximate robot frame (X forward, Y left)
            # BEV origin (img_shape[0], img_shape[1]/2) is roughly robot origin.
            robot_x = (img_shape[0] - y) * self.ym_per_pix
            robot_y = (img_shape[1]/2.0 - cx) * self.xm_per_pix
            
            # Filter by distance
            if self.min_dist <= robot_x <= self.max_dist:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = robot_x
                pose.pose.position.y = robot_y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
                center_fit_x.append((cx, y))

        return path, len(path.poses) > 0, center_fit_x
