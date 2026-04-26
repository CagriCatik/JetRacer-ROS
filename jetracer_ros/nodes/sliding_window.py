import numpy as np
import cv2

class SlidingWindow:
    def __init__(self, config):
        self.num_windows = config.get('num_windows', 9)
        self.margin = config.get('margin', 60)
        self.minpix = config.get('minpix', 40)
        self.min_lane_pixels = config.get('min_lane_pixels', 300)
        
        self.lane_width_px = config.get('lane_width_px', 230) # Fallback if only 1 lane detected
        self.left_fit = None
        self.right_fit = None

    def find_lanes(self, binary_warped):
        """Perform sliding window search to find lane lines."""
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
        
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        midpoint = int(histogram.shape[0]//2)
        
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        window_height = int(binary_warped.shape[0]//self.num_windows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.num_windows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            
            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        left_detected = len(leftx) > self.min_lane_pixels
        right_detected = len(rightx) > self.min_lane_pixels

        # Fit 2nd order polynomials
        if left_detected:
            self.left_fit = np.polyfit(lefty, leftx, 2)
        else:
            self.left_fit = None
            
        if right_detected:
            self.right_fit = np.polyfit(righty, rightx, 2)
        else:
            self.right_fit = None

        return self.left_fit, self.right_fit, out_img, left_detected, right_detected
