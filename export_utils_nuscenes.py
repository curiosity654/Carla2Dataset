"""
This file contains all the methods responsible for saving the generated data in the correct output format.

"""

from fileinput import filename
from uuid import uuid1
import numpy as np
from PIL import Image
import os
import logging
import math
import carla
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

def get_quaternion_from_euler(pitch, yaw, roll, to_rad=False):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [w,x,y,z] format
  """
  # temp for coord trans

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

def save_ref_files(OUTPUT_FOLDER, id):
    """ Appends the id of the given record to the files """
    for name in ['train.txt', 'val.txt', 'trainval.txt']:
        path = os.path.join(OUTPUT_FOLDER, name)
        with open(path, 'a') as f:
            f.write("{0:06}".format(id) + '\n')
        logging.info("Wrote reference files to %s", path)


def save_image_data(path, images, id):
    cams = ['CAM_BACK', 'CAM_BACK_RIGHT', 'CAM_FRONT_RIGHT', 'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT']
    for cam, image in zip(cams, images):
        filename = path.format(cam, id)
        logging.info("Wrote image data to %s", filename)
        image.save_to_disk(filename)

def save_bbox_image_data(filename, image):
    im = Image.fromarray(image)
    im.save(filename)

# def save_lidar_data(filename, point_cloud, format="bin"):
#     """ Saves lidar data to given filename, according to the lidar data format.
#         bin is used for KITTI-data format, while .ply is the regular point cloud format
#         In Unreal, the coordinate system of the engine is defined as, which is the same as the lidar points
#         z
#         ^   ^ x
#         |  /
#         | /
#         |/____> y
#               z
#               ^   ^ x
#               |  /
#               | /
#         y<____|/
#         Which is a right handed coordinate sylstem
#         Therefore, we need to flip the y axis of the lidar in order to get the correct lidar format for kitti.
#         This corresponds to the following changes from Carla to Kitti
#             Carla: X   Y   Z
#             KITTI: X  -Y   Z
#         NOTE: We do not flip the coordinate system when saving to .ply.
#     """
#     logging.info("Wrote lidar data to %s", filename)

#     if format == "bin":
#         point_cloud = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
#         point_cloud = np.reshape(point_cloud, (int(point_cloud.shape[0] / 4), 4))
#         point_cloud = point_cloud[:, :-1]

#         lidar_array = [[point[0], -point[1], point[2], 1.0]
#                        for point in point_cloud]
#         lidar_array = np.array(lidar_array).astype(np.float32)
#         logging.debug("Lidar min/max of x: {} {}".format(
#                       lidar_array[:, 0].min(), lidar_array[:, 0].max()))
#         logging.debug("Lidar min/max of y: {} {}".format(
#                       lidar_array[:, 1].min(), lidar_array[:, 0].max()))
#         logging.debug("Lidar min/max of z: {} {}".format(
#                       lidar_array[:, 2].min(), lidar_array[:, 0].max()))
#         lidar_array.tofile(filename)

def save_label_data(filename, datapoints):
    with open(filename, 'w') as f:
        out_str = "\n".join([str(point) for point in datapoints if point])
        f.write(out_str)
    logging.info("Wrote kitti data to %s", filename)

# def save_sample_annotation(filename, annos):
#     anno_jsons = []
#     for anno in annos:
#         anno_json = anno.to_json()
#         anno_jsons.append(anno_json)
#     extend_json(filename, anno_jsons)

def save_can_bus_data(filename, pose, imu):
    can_bus = []
    for vec in [pose.location, pose.rotation, imu["acc"], imu["vel"], imu["rot"]]:
        if type(vec) is carla.libcarla.Rotation:
            pitch = vec.pitch
            yaw = vec.yaw
            roll = vec.roll
            quat = get_quaternion_from_euler(pitch, yaw, roll, to_rad=True)
            can_bus.extend(quat)
        else:
            can_bus.append(vec.x)
            can_bus.append(vec.y)
            can_bus.append(vec.z)
    
    with open(filename, 'a') as f:
        f.write(str(can_bus).lstrip("[").rstrip("]")+"\n")

def save_rgb_image(filename, image):
    im = Image.fromarray(image)
    im.save(filename)