import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm_notebook
from os.path import split
from glob import glob


def __init__(self, bin_threshold):
        self.bin_threshold=bin_threshold
#         self.hue_min = hue_min
#         self.hue_max = hue_max
#         self.saturation = saturation
#         self.value = value
#         self.kernel_size = kernel_size
#         self.min_area = min_area
#         self.flimits = flimits

def detect(self, img):
        img_bin=self.binarization(img)
        
#         seg = self.segmentation(img)
#         seg = self.post_processing(seg)
        
#         curve = self.polygonal_curve_detection(seg)
        return img_bin
#         return seg, curve
def binarization(self,img):
        ret,th1 = cv.threshold(img,bin_threshold,255,cv.THRESH_BINARY)
        th2 = cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY,11,2)
        th3 = cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,11,2)
        titles = ['Original Image', 'Global Thresholding (v = 127)','Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
        images = [img, th1, th2, th3]
        for i in xrange(4):
            plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
            plt.title(titles[i])
            plt.xticks([]),plt.yticks([])
        plt.show()
        
        
imgs = glob('/home/daniela/Documents/SeaAI/thermal_camera/*.png')
imgs = sorted(imgs)
binarization(img) for img in imgs