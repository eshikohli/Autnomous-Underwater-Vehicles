# -*- coding: utf-8 -*-
"""
Created on Thu Mar  4 20:23:11 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import numpy as np
import utm
import random
import time
import datetime
import pytz


from pynmea2 import pynmea2
import BluefinMessages

def nmea_lat(lat_deg):
    val = np.abs(lat_deg)
    degval = np.floor(val)
    minflt = (val - degval)*60
    minval = np.floor(minflt)
    muval = int((minflt - minval)*1e6)
    return f"{int(degval):02d}{int(minval):02d}.{int(muval):06d}"

def nmea_lon(lon_deg):
    val = np.abs(lon_deg)
    degval = np.floor(val)
    minflt = (val - degval)*60
    minval = np.floor(minflt)
    muval = int((minflt - minval)*1e6)
    return f"{int(degval):03d}{int(minval):02d}.{int(muval):06d}"


class Sandshark(object):
    def __init__(self,                  
                 latlon=(0.0,0.0),
                 depth=0.0,
                 speed_knots=0.0,
                 heading=0.0,
                 rudder_position=0.0,
                 engine_speed='STOP',
                 engine_direction='AHEAD',
                 visibility=100,
                 datum=(0.0,0.0)):

        ####
        ## state components that we control
        # engine orders
        #sanity check
        if ( ( engine_speed != "STOP") and (engine_speed != "SLOW") 
            and (engine_speed != "HALF") and (engine_speed != "FULL") ):
            raise ValueError
        if ( (engine_direction != "AHEAD") and (engine_direction != "ASTERN") ):
            raise ValueError
            
        self.__timestamp = datetime.datetime.utcnow().timestamp()

        self.__engine_state = (engine_speed, engine_direction)
        
        # helm command, conning order
        self.__rudder_position = rudder_position
        
        ######
        ## state components that we observe but don't directly control
        self.__latlon = latlon
        self.__depth = depth
        self.__altitude = 10
        
        self.__speed_knots = speed_knots
        self.__speed_mps = speed_knots * 0.514444
        self.__heading = heading
        self.__battery = float('inf')
        
        self.__pitch = 0.0
        self.__roll = 0.0
        self.__yaw = 0.0
        
        self.__rudder_history = list() # history of rudder commands
        
        ######
        ## sensors
        #self.__laser = BWSI_Laser(visibility)
        #self.__camera = BWSI_Camera(visibility)
        
        ######################################
        ## external information and parameters
        self.__datum = datum
        self.__datum_position = utm.from_latlon(self.__datum[0], self.__datum[1])
        self.__position = self.__get_local_position()

        ## characteristic of vehicle; should be overwritten by a subclass
        self.__MAX_SPEED_KNOTS = 5
        self.__HARD_RUDDER_DEG = 25
        self.__FULL_RUDDER_DEG = 15
        self.__STANDARD_RUDDER_DEG = 10
        self.__MAX_TURNING_RATE = 15.64
        self.__RUDDER_COST = 0.35
        
        self.__MAX_RPM = 2500

    ##############################################################
    ## User request functions
    # update the vehicle state, dt seconds have passed since last update
    def update_state(self, dt):
        self.__timestamp += dt
        
        delta_heading = self.__MAX_TURNING_RATE * (-self.__rudder_position / self.__HARD_RUDDER_DEG) * (self.__speed_knots / self.__MAX_SPEED_KNOTS) * dt
        # adjust the delta heading based on rudder history
        delta_heading += self.__rudder_hydro_effect()
        
        final_heading = np.mod(self.__heading + delta_heading + 360.0, 360.0)
        avg_heading = np.mod( self.__heading + delta_heading/2.0 + 360.0, 360.0)
        
        # just march forward
        dx = self.__speed_mps * dt * np.sin(np.radians(avg_heading))
        dy = self.__speed_mps * dt * np.cos(np.radians(avg_heading))
        x = self.__position[0] + dx
        y = self.__position[1] + dy
        self.__position = (x,y)
        self.__latlon = utm.to_latlon(self.__position[0] + self.__datum_position[0],
                                      self.__position[1] + self.__datum_position[1],
                                      self.__datum_position[2],
                                      self.__datum_position[3])
        
        # battery usage
        self.__battery = self.__battery - np.sqrt(dx**2 + dy**2)
                
        self.__heading = final_heading
        
        # return an NMEA string
        lat_hemi = 'N'
        lat_deg = nmea_lat(self.__latlon[0])
        
        if self.__latlon[0] < 0:
            lat_hemi = 'S'
            
        lon_hemi = 'E'
        lon_deg = nmea_lon(self.__latlon[1])
        
        if self.__latlon[1] < 0:
            lon_hemi = 'W'
        
        # JRE: in strftime %f is microseconds...
        hhmmss = datetime.datetime.fromtimestamp(self.__timestamp).strftime('%H%M%S.%f')[:-4]
        msg = BluefinMessages.NVG('BF','NVG',(f'{hhmmss}',
                                              f'{lat_deg}',
                                              f'{lat_hemi}',
                                              f'{lon_deg}',
                                              f'{lon_hemi}',
                                              '0',
                                              f'{self.__altitude:.1f}',
                                              f'{self.__depth:.1f}',
                                              f'{self.__heading:.1f}',
                                              f'{self.__roll:.1f}',
                                              f'{self.__pitch:.1f}',
                                              f'{hhmmss}'))
      
        print(f'{str(msg)}\n')
        return str(msg) + '\n'
                
                        
    def engine_command(self, command):
        # break into words & force upper case
        words = command.upper().split()
        
        # sanity check on input
        if ( (len(words)<2) or 
            ( words[0] != "ENGINE") ):
            return "COMMAND"
                   
        # interpret the engine speed term
        new_engine_speed = words[1]
        if (words[1] == "STOP"):
            self.__speed_knots = 0
            new_engine_direction = self.__engine_state[1]
            self.__engine_state = (new_engine_speed, new_engine_direction)
            return command
        elif (words[1] == "SLOW"):
            speed_knots = 0.25 * self.__MAX_SPEED_KNOTS
        elif (words[1] == "HALF"):
            speed_knots = 0.5 * self.__MAX_SPEED_KNOTS
        elif (words[1] == "FULL"):
            speed_knots = self.__MAX_SPEED_KNOTS
        else:
            return "COMMAND"
        
        # interpret the engine direction term
        new_engine_direction = words[2]
        if ( (words[2] != "AHEAD") and (words[2] != "ASTERN") ):
            return "COMMAND"
        
        if (words[2] != self.__engine_state[1]):
            self.__heading = np.mod(self.__heading + 180, 360)
            
        self.__engine_state = (new_engine_speed, new_engine_direction)
        self.__speed_knots = speed_knots
        self.__speed_mps = self.__speed_knots * 0.514444
        
        return command
    
    def set_rpm(self, rpm):
        if rpm >= 0 and rpm <= self.__MAX_RPM:
            self.__speed_knots = self.__MAX_SPEED_KNOTS * rpm / self.__MAX_RPM
            self.__speed_mps = self.__speed_knots * 0.514444
        else:
            print(f"INVALID RPM REQUEST: {rpm}")
        
    def set_rudder(self, rudder):
        desired = rudder
        if np.abs(desired) <= self.__HARD_RUDDER_DEG:
            self.__rudder_position = desired
        else:
            print(f"INVALID RUDDER REQUEST: {desired}")
    
    def helm_command(self, command):
        ### Available commands:
        ## Helm commands
        # {RIGHT, LEFT} {#} DEGREES RUDDER
        # {RIGHT, LEFT} STANDARD RUDDER (standard = 15 degrees)
        # {RIGHT, LEFT} FULL RUDDER (full = 30 degrees)
        # HARD {RIGHT, LEFT} RUDDER (= 35 degrees)
        # INCREASE YOUR RUDDER TO {#} DEGREES
        # RUDDER AMIDSHIPS (= 0 degrees)
        # SHIFT YOUR RUDDER
        # MARK YOUR HEAD
        # HOW IS YOUR RUDDER
        # KEEP HER SO
        command = command.upper()
        cmd = command.split()

        if (len(cmd) < 2):
            # invalid command, request a new one
            return "COMMAND"

        if (command == "KEEP HER SO"):
            # do nothing
            return self.__reply_success(command)
        elif (command == "HOW IS YOUR RUDDER"):
            if (self.__rudder_position == 0):
                reply = "RUDDER AMIDSHIPS"
            else:
                direction = "RIGHT"
                if (self.__rudder_position < 0):
                    direction = "LEFT"
                reply = f"RUDDER {direction} {np.abs(self.__rudder_position):.1f} DEGREES"
            return reply
        elif (command == "MARK YOUR HEAD"):
            reply = f"HEADING {self.__heading:.1f} DEGREES"
            return reply
        elif (command == "SHIFT YOUR RUDDER"):
            self.__rudder_position = -self.__rudder_position
            self.__battery = self.__battery - 2*self.__RUDDER_COST*np.abs(self.__rudder_position)
            return self.__reply_success(command)
        elif (command == "RUDDER AMIDSHIPS"):
            self.__battery = self.__battery - self.__RUDDER_COST*np.abs(self.__rudder_position)
            self.__rudder_position = 0
            return self.__reply_success(command)
        elif (cmd[0] == "INCREASE"):
            return self.__parse_increase_command(command)
        elif (cmd[0] == "HARD"):
            return self.__parse_hard_command(command)
        else:
            return self.__parse_turn_command(command)
        
        
    ## accessor functions
    def get_state(self):
        auv_state = {'heading': self.__heading,
                     'rudder': self.__rudder_position,
                     'speed': self.__speed_mps,        
                     'position': self.__position}
        
        return auv_state
        
    def get_position(self):
        return self.__position
    
    def get_heading(self):
        return self.__heading
    
    def get_rudder(self):
        return self.__rudder_position
    
    def set_battery(self, val):
        self.__battery = val
    
    def get_battery(self):
        return self.__battery
    
    def get_speed(self, units='mps'):
        if units.lower() == 'mps':
            return self.__speed_mps
        else:
            return self.__speed_knots
            
    ## sensors    
    def read_laser(self, buoy_field):
        g, r = self.__laser.get_visible_buoys(self.__position, self.__heading, buoy_field)
        
        g = random.shuffle(g, list(g))
        r = random.shuffle(r, list(r))

        return g, r
    
    def read_camera(self, buoy_field):
        g, r = self.__camera.get_visible_buoys(self.__position, self.__heading, buoy_field)
        
        g = random.shuffle(g, list(g))
        r = random.shuffle(r, list(r))

        return g, r
    
    #
    ## done user requests
    ################################################################        

    ##################################################################
    ## private "helper" functions
    #
    def __parse_turn_command(self, command):
        cmd = command.split()
        
        if (len(cmd)<3):
            return "COMMAND"
        
        if (cmd[0] == "RIGHT"):
            mult = 1
        elif (cmd[0] == "LEFT"):
            mult = -1
        else:
            return "COMMAND"
        
        if (cmd[1] == "FULL"):
            if (cmd[2] == "RUDDER"):
                deg = self.__FULL_RUDDER_DEG
            else:
                return "COMMAND"
        elif (cmd[1] == "STANDARD"):
            if (cmd[2] == "RUDDER"):
                deg = self.__STANDARD_RUDDER_DEG
            else:
                return "COMMAND"
        else:
            if (len(cmd) != 4):
                return "COMMAND"
            
            if (cmd[2]=="DEGREES" and cmd[3]=="RUDDER"):
                if not cmd[1].isdigit():
                    return "COMMAND"
                deg = int(cmd[1])
            else:
                return "COMMAND"
            
            if (deg > self.__FULL_RUDDER_DEG):
                return "COMMAND"
        
        #made it through
        self.__battery = self.__battery - self.__RUDDER_COST*np.abs(self.__rudder_position-deg) 
        self.__rudder_position = mult*deg
        
        return self.__reply_success(command)
        
        
    def __parse_increase_command(self, command):
        cmd = command.split()

        if not (cmd[1]=="YOUR" and cmd[2]=="RUDDER" and cmd[3]=="TO"):
            # improper command format
            return "COMMAND"
        
        if (self.__rudder_position == 0):
            # ambiguous which direction to turn
            return "COMMAND"
        
        deg = int(cmd[4])
        
        if (deg > self.__FULL_RUDDER_DEG):
            # increasing too much
            return "COMMAND"
        
        if (deg < np.abs(self.__rudder_position)):
            # this is not increasing the rudder
            return "COMMAND"
        
        # looks like a valid command
        self.__battery = self.__battery-self.__RUDDER_COST*np.abs(self.rudder_position-deg)
        self.__rudder_position = np.sign(self.__rudder_position)*deg

        return self.__reply_success(command)
    
    def __parse_hard_command(self, command):
        cmd = command.split()
        
        if (len(cmd)<3):
            return "COMMAND"
        
        if not (cmd[2] == "RUDDER"):
            return "COMMAND"
        
        if (cmd[1] == "RIGHT"):
            self.__battery = self.__battery-self.__RUDDER_COST*np.abs(self.__rudder_position-self.__HARD_RUDDER_DEG)
            self.__rudder_position = self.__HARD_RUDDER_DEG
            return self.__reply_success(command)
        elif (cmd[1] == "LEFT"):
            self.__battery = self.__battery-self.__RUDDER_COST*np.abs(self.__rudder_position+self.__HARD_RUDDER_DEG)
            self.__rudder_position = -self.__HARD_RUDDER_DEG
            return self.__reply_success(command)
        else:
            return "COMMAND"
        
    def __reply_success(self, cmd):
        reply_string = cmd + " AYE AYE"
        return reply_string
    
    def __get_local_position(self):
        # check that datum is in the same UTM zone, if not, shift datum
        local_pos = utm.from_latlon(self.__latlon[0],
                                    self.__latlon[1],
                                    force_zone_number=self.__datum_position[2],
                                    force_zone_letter=self.__datum_position[3])
        
        return (local_pos[0]-self.__datum_position[0], local_pos[1]-self.__datum_position[1])
    
    def __update_latlon(self):
        self.__latlon = utm.to_latlon(self.__position + self.__datum_position)
        
    def __rudder_hydro_effect(self):
        for count, pos in enumerate(self.__rudder_history):
            pass
        
        return 0
                
    #    
    ## done private helpers
    ##################################