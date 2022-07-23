import json
from uuid import uuid1

from config import config_to_trans
from export_utils import *
from export_utils_nuscenes import append_json, save_can_bus_data, save_ego_pose_data, get_quaternion_from_euler
from data_utils import camera_intrinsic

class DataSave:
    def __init__(self, cfg):
        self.cfg = cfg
        self.OUTPUT_FOLDER = None
        self.LIDAR_PATH = None
        self.KITTI_LABEL_PATH = None
        self.CARLA_LABEL_PATH = None
        self.IMAGE_PATH = None
        self.CALIBRATION_PATH = None
        self.CAN_BUS_PATH = None
        self._generate_path(self.cfg["SAVE_CONFIG"]["ROOT_PATH"])
        self.captured_frame_id = self._current_captured_frame_num()
        # Nuscenes
        # TODO update scene number
        self.scene_id = 0
        self.sample_id = 0
        self.instances = []
        self._save_calibrated_sensors(cfg)


    def _generate_path(self,root_path):
        """ 生成数据存储的路径"""
        PHASE = "training"
        self.OUTPUT_FOLDER = os.path.join(root_path, PHASE)
        folders = ['calib', 'image', 'kitti_label', 'carla_label', 'velodyne', 'can_bus']
        cams = ['CAM_BACK', 'CAM_BACK_RIGHT', 'CAM_FRONT_RIGHT', 'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT']

        for folder in folders:
            directory = os.path.join(self.OUTPUT_FOLDER, folder)
            if not os.path.exists(directory):
                os.makedirs(directory)
        for cam in cams:
            directory = os.path.join(self.OUTPUT_FOLDER, 'image', cam)
            if not os.path.exists(directory):
                os.makedirs(directory)

        self.LIDAR_PATH = os.path.join(self.OUTPUT_FOLDER, 'velodyne/{0:06}.bin')
        self.KITTI_LABEL_PATH = os.path.join(self.OUTPUT_FOLDER, 'kitti_label/{0:06}.txt')
        self.CARLA_LABEL_PATH = os.path.join(self.OUTPUT_FOLDER, 'carla_label/{0:06}.txt')
        self.IMAGE_PATH = os.path.join(self.OUTPUT_FOLDER, 'image/{0}/{1:06}.png')
        self.CALIBRATION_PATH = os.path.join(self.OUTPUT_FOLDER, 'calib/{0:06}.txt')
        # Nuscenes
        self.CAN_BUS_PATH = os.path.join(self.OUTPUT_FOLDER, 'can_bus/scene_{0:06}.txt')
        self.SENSORS_PATH = os.path.join(self.OUTPUT_FOLDER, 'sensor.json')
        self.CALIBRATED_SENSORS_PATH = os.path.join(self.OUTPUT_FOLDER, 'calibrated_sensor.json')
        self.EGO_POSE_PATH = os.path.join(self.OUTPUT_FOLDER, 'ego_pose.json')
        self.SCENE_PATH = os.path.join(self.OUTPUT_FOLDER, 'scene.json')
        self.SAMPLE_PATH = os.path.join(self.OUTPUT_FOLDER, 'sample.json')
        self.INSTANCE_PATH = os.path.join(self.OUTPUT_FOLDER, 'instance.json')
        # create empty jsons
        for file in [self.EGO_POSE_PATH, self.SCENE_PATH, self.SAMPLE_PATH, self.INSTANCE_PATH]:
            with open(file, "w") as f:
                json.dump([], f)

    def _save_calibrated_sensors(self, cfg):
        sensors = []
        calibrations = []
        for s in cfg["SENSOR_CONFIG"]:
            sensor_token = uuid1().hex
            channel = s
            modality = cfg["SENSOR_CONFIG"][s]["BLUEPRINT"].split(".")[1]
            if modality == "camera":
                width = cfg["SENSOR_CONFIG"][s]["ATTRIBUTE"]["image_size_x"]
                height = cfg["SENSOR_CONFIG"][s]["ATTRIBUTE"]["image_size_y"]
                intrinsic = camera_intrinsic(width, height).tolist()
            else:
                intrinsic = []
            sensor = {
                "token": sensor_token,
                "channel": channel,
                "modality": modality
            }
            calibration = {
                "token": uuid1().hex,
                "sensor_token": sensor_token,
                "translation": cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["location"],
                "rotation": get_quaternion_from_euler(*cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["rotation"], to_rad=True),
                "camera_intrinsic": intrinsic
            }
            sensors.append(sensor)
            calibrations.append(calibration)
        with open(self.SENSORS_PATH, "w") as f:
            json.dump(sensors, f, indent=2)
        with open(self.CALIBRATED_SENSORS_PATH, "w") as f:
            json.dump(calibrations, f, indent=2)

    def _current_captured_frame_num(self):
        """获取文件夹中存在的数据量"""
        label_path = os.path.join(self.OUTPUT_FOLDER, 'kitti_label/')
        num_existing_data_files = len(
            [name for name in os.listdir(label_path) if name.endswith('.txt')])
        print("当前存在{}个数据".format(num_existing_data_files))
        if num_existing_data_files == 0:
            return 0
        answer = input(
            "There already exists a dataset in {}. Would you like to (O)verwrite or (A)ppend the dataset? (O/A)".format(
                self.OUTPUT_FOLDER))
        if answer.upper() == "O":
            logging.info(
                "Resetting frame number to 0 and overwriting existing")
            return 0
        logging.info("Continuing recording data on frame number {}".format(
            num_existing_data_files))
        return num_existing_data_files

    def save_training_files(self, data):

        lidar_fname = self.LIDAR_PATH.format(self.captured_frame_id)
        kitti_label_fname = self.KITTI_LABEL_PATH.format(self.captured_frame_id)
        carla_label_fname = self.CARLA_LABEL_PATH.format(self.captured_frame_id)
        # img_fname = self.IMAGE_PATH.format(self.captured_frame_id)
        # calib_filename = self.CALIBRATION_PATH.format(self.captured_frame_id)
        can_bus_fname = self.CAN_BUS_PATH.format(self.scene_id)

        for agent, dt in data["agents_data"].items():

            # camera_transform= config_to_trans(self.cfg["SENSOR_CONFIG"]["RGB"]["TRANSFORM"])
            # lidar_transform = config_to_trans(self.cfg["SENSOR_CONFIG"]["LIDAR"]["TRANSFORM"])

            save_ref_files(self.OUTPUT_FOLDER, self.captured_frame_id)
            save_image_data(self.IMAGE_PATH, dt["sensor_data"][7:13], self.captured_frame_id)
            save_label_data(kitti_label_fname, dt["kitti_datapoints"])
            save_label_data(carla_label_fname, dt['carla_datapoints'])
            save_can_bus_data(can_bus_fname, dt["pose"], dt["imu"], data["timestamp"])
            save_ego_pose_data(self.EGO_POSE_PATH, dt["pose"], data["timestamp"])
            # save_calibration_matrices([camera_transform, lidar_transform], calib_filename, dt["intrinsic"])
            save_lidar_data(lidar_fname, dt["sensor_data"][0])
        self.captured_frame_id += 1

    def save_instance(self, data):
        for anno in data:
            if anno.carla_id not in self.instances:
                instance = {
                    "carla_id": anno.carla_id,
                    "token": uuid1().hex,
                    "category_token": anno.category,
                    "nbr_annotations": 0,
                    "first_annotation_token": "",
                    "last_annotation_token": "" 
                }
                self.instances[str(instance.carla_id)] = (instance)
    
    def init_scene(self):
        print("scene: {}".format(self.scene_id))
        self.sample_id = 0
        self.prev_sample_token = ""
        self.sample_token = uuid1().hex
        self.next_sample_token = uuid1().hex

        self.scene = {
            "token": uuid1().hex,
            "name": "scene-{}".format(self.scene_id),
            "description": "",
            "nbr_samples": 0,
            "first_sample_token": self.sample_token,
            "last_sample_token": ""
        }
        self.scene_id += 1

    def save_scene(self):
        self.scene["nbr_samples"] = self.sample_id + 1
        self.scene["last_sample_token"] = self.sample_token
        append_json(self.SCENE_PATH, self.scene)

    def save_sample(self, timestamp):
        print("sample: {}".format(self.sample_id))
        if self.sample_id == 0:
            pass # Done in init_scene
        else:
            self.prev_sample_token = self.sample_token
            self.sample_token = self.next_sample_token
            self.next_sample_token = uuid1().hex
        sample = {
            "token": self.sample_token,
            "timestamp": timestamp,
            "scene_token": self.scene["token"],
            "next": self.next_sample_token,
            "prev": self.prev_sample_token
        }
        append_json(self.SAMPLE_PATH, sample)
        self.sample_id += 1
        return sample
    
    def save_sample_annotation(self, timestamp):
        sample_annotation = {
            "token": self.sample_token,
            "timestamp": timestamp,
            "scene_token": self.scene["token"],
            "next": self.next_sample_token,
            "prev": self.prev_sample_token
        }
        append_json(self.SAMPLE_PATH, sample)
        self.sample_id += 1
        return sample