import cv2
import numpy as np

class CameraDetector:

    def __init__(self, hue_min, hue_max, saturation, value, kernel_size, min_area, flimits):
        self.hue_min = hue_min
        self.hue_max = hue_max
        self.saturation = saturation
        self.value = value
        self.kernel_size = kernel_size
        self.min_area = min_area
        self.flimits = flimits

    def detect(self, img):
        
        seg = self.segmentation(img)
        seg = self.post_processing(seg)
        
        curve = self.polygonal_curve_detection(seg)
        
        return seg, curve

    def segmentation(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        
        color_segment = (
            np.logical_and(
                h > self.hue_min,
                h <= self.hue_max)
            if self.hue_min < self.hue_max
            else 
            np.logical_or(
                h > self.hue_min,
                h < self.hue_max))
        
        binary = np.logical_and.reduce([
            color_segment,
            s > self.saturation,
            v > self.value
            ])
        binary = binary * 255
        
        return binary.astype('u1')
        
    
    def post_processing(self, img):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_size, self.kernel_size))
        binary = cv2.erode(img, kernel)
        binary = cv2.dilate(binary, kernel)
        # binary = cv2.GaussianBlur(binary, (5,5), 2,2)
        return binary

    def polygonal_curve_detection(self, binary):
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # contours = [ cv2.convexHull(c) for c in contours ]

        if len(contours) == 0:
            return None

        contourAreas = [ cv2.contourArea(c) for c in contours ]
        contourId = np.argmax(contourAreas)
        contour = contours[contourId]

        area = cv2.contourArea(contour)
        bbox = cv2.boundingRect(contour)
        bbox = tuple(map(float, bbox))
        radius = bbox[2] / 2

        if area < self.min_area:
            return None

        ffac1 = np.abs(1 - (bbox[2] / bbox[3]))
        ffac2 = np.abs(1 - area / (np.pi * radius * radius))
        
        is_circle = ffac1 < self.flimits[0] and ffac2 < self.flimits[1]
        if not is_circle:
            return None

        centroid = ( (bbox[0] + radius, bbox[1]+ radius), radius )

        return centroid
