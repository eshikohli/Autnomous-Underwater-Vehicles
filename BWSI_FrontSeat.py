#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 16:18:55 2021

This is the simulated Sandshark front seat


@author: BWSI AUV Challenge Instructional Staff
"""
import sys

from BWSI_Sandshark import Sandshark
from BWSI_BuoyField import BuoyField
from Sandshark_Interface import SandsharkServer

import threading

import time
import datetime

import numpy as np
import matplotlib.pyplot as plt

class FrontSeat():
    # we assign the mission parameters on init
    def __init__(self, port=8000, warp=1):
        # start up the vehicle, in setpoint mode
        self.__datum = (42.3, -71.1)
        self.__vehicle = Sandshark(latlon=self.__datum,
                                   depth=1.0, 
                                   speed_knots=0.0,
                                   heading=45.0,
                                   rudder_position=0.0,
                                   engine_speed='STOP',
                                   engine_direction='AHEAD',
                                   datum=self.__datum)
        
        # front seat acts as server
        self.__server = SandsharkServer(port=port)
        self.__current_time = datetime.datetime.utcnow().timestamp()
        self.__start_time = self.__current_time
        self.__warp = warp
        
        self.__position_history = list()
        self.__doPlots = True
        
        # has heard from the backseat
        self.__isConnected = False
    
    
    def run(self):
        try:
            # start up the server
            server = threading.Thread(target=self.__server.run, args=())
            server.start()
            
            if self.__doPlots:
                fig = plt.figure()
                ax = fig.add_subplot(111)
                
                self.__simField = BuoyField(self.__datum)

                config = {'nGates': 5,
                          'gate_spacing': 5,
                          'gate_width': 2,
                          'style': 'pool_1',
                          'max_offset': 5,
                          'heading': 45}
                    
                self.__simField.configure(config)
                G, R = self.__simField.get_buoy_positions()
                green_buoys = np.asarray(G)
                red_buoys = np.asarray(R)


            count = 0
            while True:
                now = datetime.datetime.utcnow().timestamp()
                delta_time = (now-self.__current_time) * self.__warp
                msg = self.__vehicle.update_state(delta_time)
                self.__server.send_command(msg)
                self.__current_time = now
                
                msgs = self.__server.receive_mail()
                if len(msgs) > 0:
                    self.__isConnected = True
                    print("\nReceived from backseat:")
                    for msg in msgs:
                        print(f"{str(msg, 'utf-8')}")
                        self.parse_payload_command(str(msg, 'utf-8'))
                        
                    
                if self.__doPlots and self.__isConnected:
                    current_position = self.__vehicle.get_position()
                    self.__position_history.append(current_position)
                    ax.clear()
                    ax.plot(green_buoys[:,0], green_buoys[:,1], 'go')
                    ax.plot(red_buoys[:,0], red_buoys[:,1], 'ro')
                    trk = np.array(self.__position_history)
                    ax.plot(trk[-10:,0], trk[-10:,1], 'k')            
    
                    ax.set_xlim(current_position[0]-10, current_position[0]+10)
                    ax.set_ylim(current_position[1]-10, current_position[1]+10)
                    ax.set_aspect('equal')
                    
                    plt.pause(0.01)
                    plt.draw()
 
                count += 1
                time.sleep(1/self.__warp)
        except Exception as e:
            print(e)
            self.__server.cleanup()
            server.join()
            
    def parse_payload_command(self, msg):
        # the only one I care about for now is BPRMB
        print(f"Parsing {msg}")
        payld = msg.split('*')
        vals = payld[0].split(',')
        if vals[0] == '$BPRMB':
            print("Here!")
                
            # heading / rudder request
            if vals[2] != '':
                print("Here?")
                heading_mode = int(vals[7])
                if heading_mode == 0:
                    # this is a heading request!
                    print("SORRY, I DO NOT ACCEPT HEADING REQUESTS! I ONLY HAVE CAMERA SENSOR!")
                elif heading_mode == 1:
                    # this is a rudder adjustment!
                    rudder = float(vals[2])
                    print(f"SETTING RUDDER TO {rudder} DEGREES")
                    self.__vehicle.set_rudder(rudder)
                
            # speed request
            if vals[5] != '':
                speed_mode = int(vals[6])
                if speed_mode == 0:
                    RPM = int(vals[5])
                    print(f"SETTING THRUSTER TO {RPM} RPM")
                    self.__vehicle.set_rpm(RPM)
                elif speed_mode == 1:
                    # speed_request
                    print("SORRY, RPM SPEED REQUESTS ONLY! I HAVE NO GPS!")

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = 29500
        
    print(f"port = {port}")
        
    front_seat = FrontSeat(port=port)
    front_seat.run()

if __name__ == '__main__':
    main()    