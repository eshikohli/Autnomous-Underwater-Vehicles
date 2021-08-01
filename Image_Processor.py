#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 11:30:37 2021

@author: BWSI AUV Challenge Instructional Staff
"""
### JRE: for simulation only!
### MDM: added Rasperry Pi V2 camera 

import sys
import pathlib
import datetime

import time 
import numpy as np
#import picamera 
#import picamera.array

import cv2

# For simulations
from BWSI_BuoyField import BuoyField
from BWSI_Sensor import BWSI_Camera


class ImageProcessor():
    def __init__(self, camera='SIM', log_dir='./'):
        self.__camera_type = camera.upper()

        if self.__camera_type == 'SIM':
            self.__camera = BWSI_Camera(max_angle=31.1, visibility=50)
            self.__simField = None
            
        else:
            #self.__camera = picamera.PiCamera()
            self.__camera.resolution = (640, 480)
            self.__camera.framerate = 24
            time.sleep(2) # camera warmup time
            self.__image = np.empty((480*640*3,), dtype=np.uint8)

        # create my save directory
        self.__image_dir = pathlib.Path(log_dir, 'frames')
        self.__image_dir.mkdir(parents=True, exist_ok=True)

    def __detect_red_buoy(self, small_img):
    
        small_img = cv2.boxFilter(small_img, -1, (10,10)) #reduce noise with smoothing

        blue = small_img[:,:,0]
        green = small_img[:,:,1]
        red = small_img[:,:,2]

        blue_thresh = np.logical_and(blue > 150, blue < 255)
        green_thresh = np.logical_and(green > 0, green < 100)
        red_thresh = np.logical_and(red > 50, red < 255)

        bg_thresh = np.logical_and(blue_thresh,green_thresh)
        bgr_thresh = np.logical_and(bg_thresh,red_thresh)

        hsv_img = cv2.boxFilter(bgr_thresh.astype(int), -1, (50,50), normalize=False)

        scaling_factor = 255/np.max(hsv_img)

        img = hsv_img*scaling_factor
        threshold = 217
        img8 = img.astype(np.uint8)

        thresh, img_out = cv2.threshold(img8, threshold, 255, cv2.THRESH_BINARY)
        #pixels = np.argwhere(img>thresh) #find the pixels that are above the threshold

        contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #plt.imshow(bgr_thresh)

        if len(contours) > 0:
            if len(contours) > 1:
                print(contours)
            xymeans2 = contours[0].mean(axis=0)

            x_coordinate = xymeans2[0][0]
            y_coordinate = xymeans2[0][1]

            return np.array([x_coordinate,y_coordinate])
        else:
            return np.array([])

    def __detect_green_buoy(self, small_img):

        small_img = cv2.boxFilter(small_img, -1, (10,10)) #reduce noise with smoothing

        blue = small_img[:,:,0]
        green = small_img[:,:,1]
        red = small_img[:,:,2]

        blue_thresh = np.logical_and(blue > 40, blue < 255)
        green_thresh = np.logical_and(green > 140, green < 255)
        red_thresh = np.logical_and(red > 0, red < 80)

        bg_thresh = np.logical_and(blue_thresh,green_thresh)
        bgr_thresh = np.logical_and(bg_thresh,red_thresh)

        hsv_img = cv2.boxFilter(bgr_thresh.astype(int), -1, (50,50), normalize=False)

        scaling_factor = 255/np.max(hsv_img)

        img = hsv_img*scaling_factor
        threshold = 217
        img8 = img.astype(np.uint8)

        thresh, img_out = cv2.threshold(img8, threshold, 255, cv2.THRESH_BINARY)
        #pixels = np.argwhere(img>thresh) #find the pixels that are above the threshold

        contours, hierarchy = cv2.findContours(img_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #plt.imshow(bgr_thresh)

        if len(contours) > 0:
            if len(contours) > 1:
                print(contours)
            xymeans2 = contours[0].mean(axis=0)

            x_coordinate = xymeans2[0][0]
            y_coordinate = xymeans2[0][1]

            return np.array([x_coordinate,y_coordinate])
        else:
            return np.array([])

    def __sensor_position(self, pix_x, res_x): 
        sensor_pos_x = (pix_x - (res_x / 2.0)) / res_x * 3.68

        return sensor_pos_x

    def __sensor_angles(self, pos_x):
        focal_length = 3.04

        horizontal_angle = np.arctan2(pos_x,focal_length)
        horizontal_angle = np.degrees(horizontal_angle)
        
        return horizontal_angle

    def __buoy_angles(self, img):
        green_center = self.__detect_green_buoy(img)
        red_center = self.__detect_red_buoy(img)
        img_x = img.shape[1]
        green_horiz = list()
        red_horiz = list()
        if green_center.any():
            green_x = green_center[0]
            green_pos_x = self.__sensor_position(green_x, img_x)
            #green_horiz = list(__sensor_angles(green_pos_x))
            g_sa = self.__sensor_angles(green_pos_x)
            green_horiz.append(g_sa)
        if red_center.any():
            red_x = red_center[0]
            red_pos_x = self.__sensor_position(red_x, img_x)
            r_sa = self.__sensor_angles(red_pos_x)
            red_horiz.append(r_sa)
        return (green_horiz, red_horiz)
    
    def run(self, auv_state=None):
        red = list()
        green = list()
        if auv_state['heading'] is not None:
            if (self.__camera_type == 'SIM'):
                # if it's the first time through, configure the buoy field
                if self.__simField is None:
                    self.__simField = BuoyField(auv_state['datum'])
                    config = {'nGates': 5,
                              'gate_spacing': 5,
                              'gate_width': 2,
                              'style': 'pool_1',
                              'max_offset': 5,
                              'heading': 0}
                    
                    self.__simField.configure(config)
                 
                # synthesize an image
                image = self.__camera.get_frame(auv_state['position'], auv_state['heading'], self.__simField)

            elif self.__camera_type == 'PICAM':
                try:
                    self.__camera.capture(self.__image, 'bgr')
                except:
                    # restart the camera
                    #self.__camera = picamera.PiCamera()
                    self.__camera.resolution = (640, 480)
                    self.__camera.framerate = 24
                    time.sleep(2) # camera warmup time
                    
                image = self.__image.reshape((480, 640, 3))
        
            else:
                print(f"Unknown camera type: {self.__camera_type}")
                sys.exit(-10)
        
            # log the image
            fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
            cv2.imwrite(str(fn), image)
        
            green,red = self.__buoy_angles(image)
        
        return red, green
