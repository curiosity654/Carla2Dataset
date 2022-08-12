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
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [w,x,y,z] format
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

# def save_ref_files(OUTPUT_FOLDER, id):
#     """ Appends the id of the given record to the files """
#     for name in ['train.txt', 'val.txt', 'trainval.txt']:
#         path = os.path.join(OUTPUT_FOLDER, name)
#         with open(path, 'a') as f:
#             f.write("{0:06}".format(id) + '\n')
#         logging.info("Wrote reference files to %s", path)


# def save_image_data(path, images, id):
#     cams = ['CAM_BACK', 'CAM_BACK_RIGHT', 'CAM_FRONT_RIGHT', 'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT']
#     for cam, image in zip(cams, images):
#         filename = path.format(cam, id)
#         logging.info("Wrote image data to %s", filename)
#         image.save_to_disk(filename)

# def save_bbox_image_data(filename, image):
#     im = Image.fromarray(image)
#     im.save(filename)

# def save_label_data(filename, datapoints):
#     with open(filename, 'w') as f:
#         out_str = "\n".join([str(point) for point in datapoints if point])
#         f.write(out_str)
#     logging.info("Wrote kitti data to %s", filename)

# def save_sample_annotation(filename, annos):
#     anno_jsons = []
#     for anno in annos:
#         anno_json = anno.to_json()
#         anno_jsons.append(anno_json)
#     extend_json(filename, anno_jsons)

# def save_rgb_image(filename, image):
#     im = Image.fromarray(image)
#     im.save(filename)