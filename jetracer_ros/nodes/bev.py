import cv2
import numpy as np

class BEVTransform:
    def __init__(self, src_points, dst_points, out_width, out_height):
        self.src = np.float32(src_points)
        self.dst = np.float32(dst_points)
        self.out_width = out_width
        self.out_height = out_height
        
        # Compute Homography matrix and its inverse
        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)

    def warp(self, img):
        """Warp an image to Bird's Eye View."""
        return cv2.warpPerspective(img, self.M, (self.out_width, self.out_height), flags=cv2.INTER_LINEAR)

    def unwarp(self, img):
        """Warp a BEV image back to the original camera perspective."""
        return cv2.warpPerspective(img, self.Minv, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)
