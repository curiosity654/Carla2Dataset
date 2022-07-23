"""
#Values    Name      Description
----------------------------------------------------------------------------
   1    type         Describes the type of object: 'Car', 'Pedestrian', ‘Vehicles’
                     ‘Vegetation’, 'TrafficSigns', etc.
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
"""

from typing import List
from math import pi
from uuid import uuid1

class KittiDescriptor:
    """
    Kitti格式的label类
    """
    def __init__(self, type=None, bbox=None, dimensions=None, location=None, rotation_y=None, extent=None):
        self.type = type
        self.truncated = 0
        self.occluded = 0
        self.alpha = -10
        self.bbox = bbox
        self.dimensions = dimensions
        self.location = location
        self.rotation_y = rotation_y
        self.extent = extent

    def set_type(self, obj_type: str):
        self.type = obj_type

    def set_truncated(self, truncated: float):
        assert 0 <= truncated <= 1, """Truncated must be Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries """
        self.truncated = truncated

    def set_occlusion(self, occlusion: int):
        assert occlusion in range(0, 4), """Occlusion must be Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown"""
        self.occluded = occlusion

    def set_alpha(self, alpha: float):
        assert -pi <= alpha <= pi, "Alpha must be in range [-pi..pi]"
        self.alpha = alpha

    def set_bbox(self, bbox: List[int]):
        assert len(bbox) == 4, """ Bbox must be 2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates (two points)"""
        self.bbox = bbox

    def set_3d_object_dimensions(self, bbox_extent):
        # Bbox extent consists of x,y and z.
        # The bbox extent is by Carla set as
        # x: length of vehicle (driving direction)
        # y: to the right of the vehicle
        # z: up (direction of car roof)
        # However, Kitti expects height, width and length (z, y, x):
        height, width, length = bbox_extent.z, bbox_extent.x, bbox_extent.y
        # Since Carla gives us bbox extent, which is a half-box, multiply all by two
        self.extent = (height, width, length)
        self.dimensions = "{} {} {}".format(2*height, 2*width, 2*length)

    def set_3d_object_location(self, obj_location):
        """
            将carla相机内目标中心点坐标转换为kitti格式的中心点坐标
            carla x y z
            kitti z x -y
            z
            ▲   ▲ x
            |  /
            | /
            |/____> y
            However, the camera coordinate system for KITTI is defined as
                ▲ z
               /
              /
             /____> x
            |
            |
            |
            ▼
            y
            Carla: X   Y   Z
            KITTI:-X  -Y   Z
        """
        # Object location is four values (x, y, z, w). We only care about three of them (xyz)
        x, y, z = [float(x) for x in obj_location][0:3]
        assert None not in [
            self.extent, self.type], "Extent and type must be set before location!"

        if self.type == "Pedestrian":
            # Since the midpoint/location of the pedestrian is in the middle of the agent, while for car it is at the bottom
            # we need to subtract the bbox extent in the height direction when adding location of pedestrian.
            z -= self.extent[0]

        self.location = " ".join(map(str, [y, -z, x]))

    def set_rotation_y(self, rotation_y: float):
        assert - \
            pi <= rotation_y <= pi, "Rotation y must be in range [-pi..pi] - found {}".format(
                rotation_y)
        self.rotation_y = rotation_y


    def __str__(self):
        """ Returns the kitti formatted string of the datapoint if it is valid (all critical variables filled out), else it returns an error."""
        if self.bbox is None:
            bbox_format = " "
        else:
            bbox_format = " ".join([str(x) for x in self.bbox])

        # kitti目标检测数据的标准格式
        return "{} {} {} {} {} {} {} {}".format(self.type, self.truncated, self.occluded,
                                                         self.alpha, bbox_format, self.dimensions, self.location,
                                                         self.rotation_y)

"""
#Values    Name      Description
----------------------------------------------------------------------------
   1    type                 Describes the type of object: 'Car', 'Pedestrian', ‘Vehicles’
                            ‘Vegetation’, 'TrafficSigns', etc.
   3    velocity             velocity of the object, returns three absolute values of the components x, y and z.
   3    acceleration         acceleration of the objects, returns three absolute values of the components x, y and z.
   3    angular_velocity     angular_velocity of the objects, returns three absolute values of the components x, y and z.
"""

class CarlaDescriptor:
    def __init__(self):
        self.type = None
        self.velocity = None
        self.acceleration = None
        self.angular_velocity = None


    def set_type(self, obj_type: str):
        self.type = obj_type

    def set_velocity(self, velocity):
        self.velocity = velocity

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration

    def set_angular_velocity(self, angular_velocity):
        self.angular_velocity = angular_velocity

    def __str__(self):
        return "{} {} {} {}".format(self.type, self.velocity, self.acceleration, self.angular_velocity)

class NuscenesDescriptor:
    """
    Nuscenes格式的label类
    """
    def __init__(self):
        self.carla_id = 0
        self.token = uuid1().hex
        self.sample_token = ""
        self.instance_token = ""
        self.attribute_tokens = []
        self.visibility_token = ""
        self.translation = []
        self.size = []
        self.rotation = []
        self.num_lidar_pts = 0
        self.next = ""
        self.prev = ""
    
    def set_carla_id(self, carla_id: int):
        self.carla_id = carla_id

    def set_sample_token(self, sample_token: str):
        self.sample_token = sample_token

    def set_instance_token(self, instance_token: str):
        self.instance_token = instance_token

    def set_attribute_tokens(self, attribute_tokens: list):
        self.attribute_tokens = attribute_tokens

    def set_visibility_token(self, visibility_token: str):
        self.visibility_token = visibility_token

    def set_translation(self, translation: list):
        assert len(translation) == 3
        self.translation = translation

    def set_size(self, size: list):
        assert len(size) == 3
        self.size = size

    def set_rotation(self, rotation: list):
        assert len(rotation) == 4
        self.rotation = rotation

    def set_num_lidar_pts(self, num_lidar_pts: int):
        self.num_lidar_pts = num_lidar_pts

    def set_next(self, next: str):
        self.next = next

    def set_prev(self, prev: str):
        self.prev = prev

    def to_json(self):
        sample_annotation = {
            "token": self.token,
            "sample_token": self.sample_token,
            "instance_token": self.instance_token,
            "attribute_tokens": self.attribute_tokens,
            "visibility_token": self.visibility_token,
            "translation": self.translation,
            "size": self.size,
            "rotation": self.rotation,
            "num_lidar_pts": self.num_lidar_pts,
            "num_radar_pts": 0, # reserve for further use
            "next": self.next,
            "prev": self.prev
        }

        return sample_annotation