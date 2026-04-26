import cv2
import numpy as np

class LaneDetector:
    def __init__(self, config):
        self.use_hls = config.get('use_hls', True)
        self.hls_s_min = config.get('hls_s_min', 80)
        self.hls_s_max = config.get('hls_s_max', 255)
        self.hls_l_min = config.get('hls_l_min', 120)
        self.hls_l_max = config.get('hls_l_max', 255)
        
        self.use_gradient = config.get('use_gradient', True)
        self.grad_min = config.get('gradient_min', 30)
        self.grad_max = config.get('gradient_max', 120)

    def process(self, img):
        """Apply color and gradient thresholds to extract a binary mask of lane lines."""
        masks = []

        if self.use_hls:
            hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
            s_channel = hls[:,:,2]
            l_channel = hls[:,:,1]
            
            s_binary = np.zeros_like(s_channel)
            s_binary[(s_channel >= self.hls_s_min) & (s_channel <= self.hls_s_max)] = 1
            
            l_binary = np.zeros_like(l_channel)
            l_binary[(l_channel >= self.hls_l_min) & (l_channel <= self.hls_l_max)] = 1
            
            masks.append(s_binary & l_binary)

        if self.use_gradient:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            abs_sobelx = np.absolute(sobelx)
            scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
            
            sxbinary = np.zeros_like(scaled_sobel)
            sxbinary[(scaled_sobel >= self.grad_min) & (scaled_sobel <= self.grad_max)] = 1
            masks.append(sxbinary)

        # Combine masks
        combined_binary = np.zeros_like(img[:,:,0])
        if masks:
            for mask in masks:
                combined_binary[(mask == 1)] = 1
                
        return combined_binary
