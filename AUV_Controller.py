#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 12:05:08 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import numpy as np
import math

class AUVController():
    def __init__(self):
        
        # initialize state information
        self.__heading = None
        self.__speed = 1000
        self.__rudder = None
        self.__position = None
        self__auv_state = None
        
        # assume we want to be going the direction we're going for now
        self.__desired_heading = None

        self.__green_buoys = None
        self.__red_buoys = None

    '''
        self.__iterations = 0 #number of iterations that it does not see any buoys
        self.__prev_gnext = None
        self.__prev_rnext = None
        self.__on_first_iteration = True

    def initialize(self, auv_state):
        self.__heading = auv_state['heading']
        self.__speed = auv_state['speed']
        self.__rudder = auv_state['rudder']
        self.__position = auv_state['position']
        
        # assume we want to be going the direction we're going for now
        self.__desired_heading = auv_state['heading']
    '''
    
    def decide(self, auv_state, green_buoys, red_buoys, sensor_type='POSITION'):
        
        self.__auv_state = auv_state
        self.__heading = auv_state['heading']
        self.__position = auv_state['position']

        # determine what heading we want to go
        if sensor_type.upper() == 'POSITION': # known positions of buoys
            self.__desired_heading = self.__heading_to_position(green_buoys, red_buoys)
        elif sensor_type.upper() == 'ANGLE': # camera sensor
            self.__desired_heading = self.__heading_to_angle(green_buoys, red_buoys)
        
        # determine whether and what command to issue to desired heading               
        cmd = self.__select_command()
        
        return cmd
        
    # return the desired heading to a public requestor
    def get_desired_heading(self):
        return self.__desired_heading
    
    ### Private member functions
        
    # calculate the heading we want to go to reach the gate center
    def __heading_to_position(self, gnext, rnext):
        # center of the next buoy pair
        gate_center = ((gnext[0]+rnext[0])/2.0, (gnext[1]+rnext[1])/2.0)
        
        # heading to gate_center
        tgt_hdg = np.mod(np.degrees(np.arctan2(gate_center[0]-self.__position[0],
                                               gate_center[1]-self.__position[1]))+360,360)
        
        return tgt_hdg
    
    def __heading_to_angle(self, gnext, rnext):
        # relative angle to the center of the next buoy pair

        relative_angle = None #default is to maintain the same heading
        #iter_threshold = 25

        #if len(gnext) > 1 or len(rnext) > 1: #if there are multiple red or green buoys, find the buoys closest to the AUV
            #as the AUV is moving, the buoys whose angles change more rapidly are closer
            #the pair of buoys also should have a similar changes in angle
            #relative_angle = self.__isolate_buoys(gnext,rnext)  

        if len(gnext) > 0 and len(rnext) > 0:
            relative_angle = (gnext[0] + rnext[0]) / 2.0
            #self.__on_first_iteration = 0
            #self.__prev_gnext = None
            #self.__prev_rnext= None
            #self.__iterations = 0
        elif len(gnext) > 0: #can only see one green buoy
            relative_angle = gnext[0]
            #self.__on_first_iteration = 0
            #self.__prev_gnext = None
            #self.__prev_rnext= None
        elif len(rnext) > 0:
            relative_angle = rnext[0]
            #self.__on_first_iteration = 0
            #self.__prev_gnext = None
            #self.__prev_rnext= None
        else:
            relative_angle = 0.0
            #self.__on_first_iteration = 0
            #self.__prev_gnext = None
            #self.__prev_rnext= None

        '''
        if len(gnext) == 0 and len(rnext) == 0:
            self.__iterations += 1

        if (self.__iterations > iter_threshold):
            relative_angle = 35
        
        if len(gnext) > 0 or len(rnext) > 0: #can see a buoy
            self.__iterations = 0
        '''

        '''
        else:
            self.__iterations += 1

        if self.__iterations >= iter_threshold:
            self.__searching = True
        
        self.__searching = False
        
        if self.__searching:
            if (len(gnext) < 1 and len(rnext) < 1): #no buoy in sight
                relative_angle = 20 #start turning
            else: #one buoy in sight
                self.__searching = False #no longer searching for a buoy
                self.__iterations = 0 #reset the search iterations
        '''

        # heading to center of the next buoy pair
        if self.__heading is not None:
            tgt_hdg = np.mod(self.__heading + relative_angle,360)
        else:
            tgt_hdg = np.mod(relative_angle,360)
        
        return tgt_hdg

    # choose a command to send to the front seat
    def __select_command(self):
        # Unless we need to issue a command, we will return None
        cmd = None
        max_angle = 25.0
        
        # determine the angle between current and desired heading
        if self.__heading is not None:
            delta_angle = self.__desired_heading - self.__heading
        else:
            delta_angle = self.__desired_heading
        if delta_angle > 180: # angle too big, go the other way!
            delta_angle = delta_angle - 360
        if delta_angle < -180: # angle too big, go the other way!
            delta_angle = delta_angle + 360
        
        # which way do we have to turn
        if delta_angle>2: # need to turn to right!
            #if self.__rudder >= 0: # rudder is turning the other way!
            degrees = -math.ceil(delta_angle)
            if degrees < -max_angle:
                degrees = -max_angle
            cmd = "turn " + str(degrees)
        elif delta_angle<-2: # need to turn to left!
            degrees = -math.ceil(delta_angle)
            if degrees > max_angle:
                degrees = max_angle
            cmd = "turn " + str(degrees)
        else: #close enough!
            cmd = ""
        
        return cmd + ";thruster " + str(self.__speed)

    def __distance(self, x1, y1, x2, y2):
        return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

    def __isolate_buoys(self, gnext, rnext):

        relative_angle = 0.0

        largest_g_angle_diff = 0.0
        closest_g = None

        largest_r_angle_diff = 0.0
        closest_r = None

        if self.__prev_gnext is not None: #not on the first iteration of having multiple buoys

            for i in range(0,len(gnext)): #for each g_angle in gnext

                current_g = gnext[i]
                if i < len(self.__prev_gnext):
                    delta_angle = current_g - self.__prev_gnext[i] #current minus prev
                
                    #need the smaller change in angle

                    if delta_angle > 180: # angle too big, go the other way!
                        delta_angle = delta_angle - 360
                    if delta_angle < -180: # angle too big, go the other way!
                        delta_angle = delta_angle + 360
                    
                    if abs(delta_angle) > largest_g_angle_diff:
                        largest_g_angle_diff = abs(delta_angle)
                        closest_g = gnext[i]
        
        if self.__prev_rnext is not None: 

            for i in range(0,len(rnext)): #for each g_angle in gnext

                current_r = rnext[i]
                if i < len(self.__prev_rnext):
                    delta_angle = current_r - self.__prev_rnext[i] #current minus prev
                    
                    #need the smaller change in angle

                    if delta_angle > 180: # angle too big, go the other way!
                        delta_angle = delta_angle - 360
                    if delta_angle < -180: # angle too big, go the other way!
                        delta_angle = delta_angle + 360
                    
                    if abs(delta_angle) > largest_r_angle_diff:
                        largest_r_angle_diff = abs(delta_angle)
                        closest_r = rnext[i]

        if closest_g is not None and closest_r is not None: #there is a closest green and red buoy
            if abs(largest_g_angle_diff - largest_r_angle_diff) < 10: #angles are close enough
                relative_angle = (closest_g + closest_r) / 2.0
        elif closest_g is not None and closest_r is None:
            relative_angle = closest_g
        elif closest_r is not None and closest_g is None:
            relative_angle = closest_r
        else:
            relative_angle = 0.0

        self.__prev_gnext = gnext
        self.__prev_rnext= rnext

        return relative_angle
