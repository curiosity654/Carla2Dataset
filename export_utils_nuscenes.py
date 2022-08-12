"""
This file contains all the methods responsible for saving the generated data in the correct output format.

"""

import numpy as np
import math
import json

def append_json(filename, dict):
    with open(filename, "r") as f:
        content = json.load(f)
        content.append(dict)
        with open(filename, "w") as f:
            json.dump(content, f, indent=2)

def extend_json(filename, list):
    with open(filename, "r") as f:
        content = json.load(f)
        content.extend(list)
        with open(filename, "w") as f:
            json.dump(content, f, indent=2)

def get_quaternion_from_euler(pitch, yaw, roll, to_rad=False, to_right=True):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
    :param to_rad: Convert the input from degree to rad
    :param to_right: Convert the rotation from left-handed coords to right-handed coords
 
  Output
    :return qw, qx, qy, qz: The orientation in quaternion [w,x,y,z] format
  """
  # temp for coord trans
  if to_right:
    yaw = -yaw
    roll = -roll

  def degrees_to_radians(degrees):
    return degrees * math.pi / 180

  if to_rad:
    pitch = degrees_to_radians(pitch)
    yaw = degrees_to_radians(yaw)
    roll = degrees_to_radians(roll)

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qw, qx, qy, qz]