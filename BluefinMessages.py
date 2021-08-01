#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 10:27:35 2021

@author: BWSI AUV Challenge Instructional Staff
"""

from pynmea2 import pynmea2
from pynmea2.pynmea2 import TalkerSentence
from pynmea2.pynmea2.nmea_utils import timestamp

_TIMESTAMP_ = (
        ('Timestamp', 'timestamp', timestamp),
    )

def timestamp_plus_string(data_name, short_name):
    fields = (
        ('Timestamp', 'timestamp', timestamp),
        (data_name, short_name),
    )
    return fields

def timestamp_plus_int(data_name, short_name):
    fields = (
        ('Timestamp', 'timestamp', timestamp),
        (data_name, short_name, int),
    )
    return fields

def timestamp_plus_float(data_name, short_name):
    fields = (
        ('Timestamp', 'timestamp', timestamp),
        (data_name, short_name, float),
    )
    return fields



# ---------------------------------------------------------------------------- #
# ----Command messages from vehicle to payload-------------------------------- #
# ---------------------------------------------------------------------------- #
class MSC(TalkerSentence):
    """ Payload mission command
    """
    fields = timestamp_plus_string('Payload mission command', 'mission_command')

class SHT(TalkerSentence):
    """ Payload shutdown
    """
    fields = _TIMESTAMP_


class BDL(TalkerSentence):
    """ Begin data logging
    """
    fields = timestamp_plus_string('Current mission step', 'mission_step')


class SDL(TalkerSentence):
    """ Stop data logging
    """
    fields = timestamp_plus_string('Current mission step', 'mission_step')
    

class TOP(TalkerSentence):
    """ Topside (not implemented by Bluefin)
    """
    fields = timestamp_plus_int('Method of transport', 'transport_method')

class DVT(TalkerSentence):
    """ Begin/end DVL external trigger
    """
    fields = timestamp_plus_int('Payload start/stop', 'trigger_flag')

class VER(TalkerSentence):
    """ Vehicle interface version
    """
    fields = timestamp_plus_string('Version number', 'version_number')
    
# ---------------------------------------------------------------------------- #
# ----Status messages from vehicle to payload-------------------------------- #
# ---------------------------------------------------------------------------- #
class NVG(TalkerSentence):
    """ Navigation update
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Latitude', "latitude_degrees", float),
            ('Hemisphere N/S', "latitude_hemisphere"),
            ('Longitude', "latitude_degrees", float),
            ('Hemisphere E/W', "longitude_hemisphere"),
            ('Position quality', "gps_available", int),
            ('Altitude', "altitude_m", float),
            ('Depth', "depth_m", float),
            ('Heading', "heading_deg", float),
            ('Roll', "roll_deg", float),
            ('Pitch', "pitch_deg", float),
            ('Timestamp of Fix', 'fix_timestamp', timestamp),
         )
    
class NVR(TalkerSentence):
    """ Velocity and rate update
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('East velocity', "east_velocity", float),
            ('North velocity', "north_velocity", float),
            ('Down velocity', "down_velocity", float),
            ('Pitch rate', "pitch_rate", float),
            ('Roll rate', "roll_rate", float),
            ('Yaw rate', "yaw_rate", float),
         )

class TEL(TalkerSentence):
    """ Telemetry status (not implemented by Bluefin)
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Acoustic upload capacity', "acoustic_kb_upload_remain", float),
            ('Acoustic upload rate', "acoustic_kb_upload", float),
            ('Acoustic download capacity', "acoustic_kb_upload_remain", float),
            ('Acoustic download rate', "acoustic_kb_download", float),
            ('RF status', "rf_status", int),
            ('Ethernet status', "ethernet_status", int),
         )
    

class SVS(TalkerSentence):
    """ Sound velocity
    """
    fields = timestamp_plus_float('Sound speed', 'sound_speed')
    
    
class RCM(TalkerSentence):
    """ Raw compass data
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Compass number', 'compass_number', int),
            ('Heading', 'heading_deg', float),
            ('Pitch', 'pitch_deg', float),
            ('Roll', 'roll_deg', float),
            ('Timestamp of data', 'data_timestamp', timestamp),
        )
    
class RDP(TalkerSentence):
    """ Raw depth sensor data
    """
    fields = timestamp_plus_float('Pressure', 'pressure_kpa')
    
class RVL(TalkerSentence):
    """ Raw vehicle speed
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Thruster RPM', 'thruster_rpm', float),
            ('Speed', 'speed_mps', float),
        )
    
class RBS(TalkerSentence):
    """ Battery voltage
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Battery number', 'battery_number', int),
            ('Stack voltage', 'stack_voltage', float),
            ('Minimum cell voltage', 'cell_voltage_min', float),
            ('Maximum cell voltage', 'cell_voltage_max', float),
            ('Maximum temperature', 'temp_C_max', float),
            ('Battery capacity', 'capacity_kwh', float),
            ('Percent charge', 'percent_charge', float),
        )
    
class MBS(TalkerSentence):
    """ Begin new behavior
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Dive file', 'dive_file'),
            ('Behavior number', 'behavior_number', int),
            ('Behavior identifier', 'behavior_identifier', int),
            ('Behavior name', 'behavior_name'),
        )
    
class MBE(TalkerSentence):
    """ End behavior
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Dive file', 'dive_file'),
            ('Behavior number', 'behavior_number', int),
            ('Behavior identifier', 'behavior_identifier', int),
            ('Behavior name', 'behavior_name'),
        )
    
    
class MIS(TalkerSentence):
    """ Mission status
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Dive file', 'dive_file'),
            ('Mission status', 'mission_status'),
            ('Status details', 'status_details'),
        )
    
class ERC(TalkerSentence):
    """ Elevator and rudder data
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Current elevator angle', 'elevator_current_deg', float),
            ('Current rudder angle', 'rudder_current_deg', float),
            ('Target elevator angle', 'elevator_target_deg', float),
            ('Target rudder angle', 'rudder_target_deg', float),
        )

class DVL(TalkerSentence):
    """ Raw DVL data
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Forward speed', 'forward_speed_mps', float),
            ('Starboard speed', 'starboard_speed_mps', float),
            ('Down speed', 'down_speed_mps', float),
            ('Beam range 1', 'beam_range_1'),
            ('Beam range 2', 'beam_range_2'),
            ('Beam range 3', 'beam_range_3'),
            ('Beam range 4', 'beam_range_4'),
            ('Temperature', 'temperature_c', float),
            ('Timestamp of data', 'data_timestamp', timestamp),
        )
    
### SKIP: BFDV2

class IMU(TalkerSentence):
    """ Raw IMU data
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Angular rate x', 'angular_rate_x', float),
            ('Angular rate y', 'angular_rate_y', float),
            ('Angular rate z', 'angular_rate_z', float),
            ('Acceleration x', 'acceleration_x', float),
            ('Acceleration y', 'acceleration_y', float),
            ('Acceleration z', 'acceleration_z', float),
            ('Timestamp of data', 'data_timestamp', timestamp),
        )

class CTD(TalkerSentence):
    """ Raw CTD data
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Conductivity', 'conductivity_uS_cm', float),
            ('Temperature', 'temperature_c', float),
            ('Pressure', 'pressure_kpa', float),
            ('Timestamp of data', 'data_timestamp', timestamp),
        )

### SKIP: BFRNV
### SKIP: BFPIT
### SKIP: BFCNV

class PLN(TalkerSentence):
    """ Mission plan element
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Behavior number', 'behavior_number', int),
            ('Behavior identifier', 'behavior_identifier', int),
            ('Sequential index', 'sequential_index', int),
            ('Element type', 'element_type', int),
            ('Latitude', 'latitude_deg', float),
            ('Hemisphere N/S', 'latitude_hemisphere', float),
            ('Longitude', 'longitude_deg', float),
            ('Undefined float', 'undefined_float', float),
            ('Undefined string 1', 'undefined_string_1'),
            ('Undefined string 2', 'undefined_string_2'),        
        )
    
class ACK(TalkerSentence):
    """ Acknowledgment
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Command name', 'command_name'),
            ('Timestamp of command', 'command_timestamp', timestamp),
            ('Behavior identifier', 'behavior_identifier', int),
            ('Acknowledgment status code', 'ack_status_code', int),
            ('Undefined int', 'undefined_int', int),
            ('Further details', 'ack_details'),
        )
    
class TRM(TalkerSentence):
    """ Trim status
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Status code', 'status_code', int),
            ('Error code', 'error_code', int),
            ('Status message', 'status_message'),
            ('Behavior identifier', 'behavior_identifier', float),
            ('Pitch trim estimate', 'pitch_trim_deg', float),
            ('Roll trim estimate', 'roll_trim_deg', float),
        )    

class BOY(TalkerSentence):
    """ Buoyancy status
    """
    fields = (
            ('Timestamp', 'timestamp', timestamp),
            ('Status code', 'status_code', int),
            ('Error code', 'error_code', int),
            ('Status message', 'status_message'),
            ('Behavior identifier', 'behavior_identifier', float),
            ('Buoyancy estimate', 'buoyancy_N', float),
        )    


class DTL(TalkerSentence):
    """ Backseat control
    """
    fields = timestamp_plus_int('Backseat control status', 'backseat_control')
     
# ---------------------------------------------------------------------------- #
# ----Messages from payload to vehicle---------------------------------------- #
# ---------------------------------------------------------------------------- #
# These will be function calls because some have the same 3-letter code
# as a BF message but are BP messages, with a different format. Also to control
# the print format
def checksum(my_str):
    cksum = 0
    # checksum is bitwise XOR of ascii
    for s in my_str:
        cksum ^= ord(s)
    
    return cksum

def str_to_cmd(msg_str):
    return f"${msg_str}*{hex(checksum(msg_str))[2:]}\n"

def BPLOG(identifier='ALL', log_request='ON'):
    """ Logging control
    """
    assert(len(identifier.upper()) == 3)
    assert(log_request.upper() == 'ON' or log_request.upper() == 'OFF')
    msg_str = f'BPLOG,{identifier.upper()},{log_request.upper()}'
    
    return str_to_cmd(msg_str)

def BPSTS(timestamp, status=1, msg=''):
    """ Payload status
    """
    assert(status==0 or status==1)
    msg_str = f'BPSTS,{timestamp},{status},{msg}'
    
    return str_to_cmd(msg_str)

def BPTOP(timestamp, method, msg):
    """ Request to send data topside
    """
    assert(isinstance(method, int) and method>=0 and method<4)
    msg_str = f'BPTOP,{timestamp},{method},{msg},0'
    
    return str_to_cmd(msg_str)

# SKIP: BPDVR

def BPTRK(timestamp, identifier,
          start_lat_deg, start_lat_hemi,
          start_lon_deg, start_lon_hemi,
          end_lat_deg, end_lat_hemi,
          end_lon_deg, end_lon_hemi,
          vertical_mode, depth_alti,
          speed, speed_mode, interrupt_code):
    """ Request additional trackline
    """
    assert(vertical_mode==0 or vertical_mode==1)    
    assert(speed_mode==0 or speed_mode==1)  
    assert(interrupt_code==0 or interrupt_code==1)  
    
    msg_str = f'BPTRK,{timestamp},{identifier:.5d},'
    msg_str += f'{start_lat_deg:.2f},{start_lat_hemi},{start_lon_deg:.2f},{start_lon_hemi},'
    msg_str += f'{end_lat_deg:.2f},{end_lat_hemi},{end_lon_deg:.2f},{end_lon_hemi},'
    msg_str += f'{vertical_mode},{depth_alti:.1f},{speed:.1f},{speed_mode},{interrupt_code}'
    
    return str_to_cmd(msg_str)
    
def BPTRC(timestamp, identifier,
          center_lat_deg, center_lat_hemi,
          center_lon_deg, center_lon_hemi,
          radius, num_orbits,
          vertical_mode, depth_alti, 
          speed, speed_mode, 
          entry_angle, interrupt_code):
    """ Request additional trackcircle
    """
    assert(vertical_mode==0 or vertical_mode==1)    
    assert(speed_mode==0 or speed_mode==1)  
    assert(interrupt_code==0 or interrupt_code==1)  

    msg_str = f'BPTRC,{timestamp},{identifier:.5d},'
    msg_str += f'{center_lat_deg:.2f},{center_lat_hemi},{center_lon_deg:.2f},{center_lon_hemi},'
    msg_str += f'{radius:.1f},{num_orbits:.1f},{vertical_mode},{depth_alti:.1f},{speed:.1f},{speed_mode},'
    msg_str += f'{entry_angle:.1f},{interrupt_code}'

    return str_to_cmd(msg_str)

# SKIP: BPRGP

def BPRCN(timestamp, identifier):
    """ Cancel requested behavior
    """
    
    msg_str = f'BPRCN,{timestamp},{identifier:.5d}'
    
    return str_to_cmd(msg_str)

def BPRCA(timestamp):
    """ Cancel all requested behaviors
    """
    
    msg_str = f'BPRCA,{timestamp}'
    
    return str_to_cmd(msg_str)

def BPRCB(timestamp):
    """ Cancel current behavior
    """
    
    msg_str = f'BPRCB,{timestamp},0'
    
    return str_to_cmd(msg_str)

def BPRCE(timestamp):
    """ Cancel current mission element
    """
    
    msg_str = f'BPRCE,{timestamp},0'
    
    return str_to_cmd(msg_str)

def BPRMB(timestamp, heading='', 
          depth='', depth_mode='',
          speed='', speed_mode='',
          horiz_mode=0):
    assert(depth_mode==0 or depth_mode==1 or depth_mode=='')
    assert(speed_mode==0 or speed_mode==1 or speed_mode=='')
    assert(horiz_mode==0 or horiz_mode==1)
    
    if depth != '':
        depth = f'{depth:.1f}'
    if speed != '':
        speed = f'{speed}'
    msg_str = f'BPRMB,{timestamp},{heading},{depth},{depth_mode},'
    msg_str += f'{speed},{speed_mode},{horiz_mode}'
    
    return str_to_cmd(msg_str)
    
# BPEMB
# Skip: BPCTD

def BPABT(timestamp, msg=''):
    """ Abort mission
    """
    
    msg_str = f'BPABT,{timestamp},{msg}'
    
    return str_to_cmd(msg_str)
    
def BPKIL(timestamp, msg=''):
    """ Kill mission
    """
    
    msg_str = f'BPKIL,{timestamp},{msg}'
    
    return str_to_cmd(msg_str)
    
def BPMSG(timestamp, msg=''):
    """ Log message
    """
    msg_str = f'BPMSG,{timestamp},{msg}'
    
    return str_to_cmd(msg_str)

def BPRMP(timestamp):
    """ Request mission plan
    """
    msg_str = f'BPRMP,{timestamp}'
    
    return str_to_cmd(msg_str)
    
# BPNPU

def BPSIL(timestamp, mode=0):
    """ Silent mode
    """
    assert(mode==0 or mode==1)

    msg_str = f'BPSIL,{timestamp},{mode}'
    
    return str_to_cmd(msg_str)
    
# BPTRM
# BPBOY

def BPVER(timestamp, version):
    """ payload interface version
    """
    msg_str = f'BPVER,{timestamp},{version}'
    
    return str_to_cmd(msg_str)
    
    
def BPLIT(timestamp, onoff=0):
    """ strobe light control (HAUV only?)
    """
    assert(onoff==0 or onoff==1)
    
    msg_str = f'BPLIT,{timestamp},1,{onoff}'
    
    return str_to_cmd(msg_str)
    