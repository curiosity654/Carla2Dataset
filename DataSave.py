import json
from unicodedata import category
from uuid import uuid1

from config import config_to_trans
from export_utils import *
from export_utils_nuscenes import append_json, extend_json, save_can_bus_data, get_quaternion_from_euler
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
        self.ROOT_PATH = self.cfg["SAVE_CONFIG"]["ROOT_PATH"]
        self._generate_path(self.ROOT_PATH)
        self.captured_frame_id = self._current_captured_frame_num()
        # Nuscenes
        self.scene_id = 0
        self.sample_id = 0
        self.instances = {}
        self._save_calibrated_sensors(cfg)
        self._save_category(cfg)


    def _generate_path(self,root_path):
        """ 生成数据存储的路径"""
        PHASE = "training"
        VERSION = "mini"
        self.OUTPUT_FOLDER = os.path.join(root_path, PHASE)
        folders = ['calib', 'image', 'kitti_label', 'carla_label', 'velodyne', 'can_bus', VERSION]
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
        self.SENSORS_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'sensor.json')
        self.CALIBRATED_SENSORS_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'calibrated_sensor.json')
        self.EGO_POSE_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'ego_pose.json')
        self.CATE_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'category.json')
        self.SCENE_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'scene.json')
        self.SAMPLE_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'sample.json')
        self.SAMPLE_DATA_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'sample_data.json')
        self.SAMPLE_ANNO_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'sample_annotation.json')
        self.INSTANCE_PATH = os.path.join(self.OUTPUT_FOLDER, VERSION, 'instance.json')
        # create empty jsons
        for file in [
            self.EGO_POSE_PATH, 
            self.SCENE_PATH, 
            self.SAMPLE_PATH, 
            self.SAMPLE_DATA_PATH,
            self.SAMPLE_ANNO_PATH,
            self.INSTANCE_PATH, 
        ]:
            with open(file, "w") as f:
                json.dump([], f)

    def _save_category(self, cfg):
        categorys = []
        for k, v in cfg["ANNOTATE_CATEGORIES"].items():
            categorys.append(
                {
                    "token": v,
                    "name": k,
                    "description": ""
                }
            )
        with open(self.CATE_PATH, "w") as f:
            json.dump(categorys, f, indent=2)

    # TODO change func name to a more generalized one
    def _save_calibrated_sensors(self, cfg):
        sensors = []
        calibrations = []
        self.last_sample_data_tokens = []
        for s in cfg["SENSOR_CONFIG"]:
            sensor_token = uuid1().hex
            calib_token = uuid1().hex
            channel = s
            modality = cfg["SENSOR_CONFIG"][s]["BLUEPRINT"].split(".")[1]
            sensor = {
                "token": sensor_token,
                "calib_token": calib_token, # TODO for convenience
                "channel": channel,
                "modality": modality
            }
            if modality == "camera":
                width = cfg["SENSOR_CONFIG"][s]["ATTRIBUTE"]["image_size_x"]
                height = cfg["SENSOR_CONFIG"][s]["ATTRIBUTE"]["image_size_y"]
                fov = cfg["SENSOR_CONFIG"][s]["ATTRIBUTE"]["fov"]
                # rotation = cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["quat"]
                # euler = cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["rotation"]
                # rotation = get_quaternion_from_euler(euler[0], euler[1], euler[2], to_rad=True)
                intrinsic = camera_intrinsic(width, height, fov).tolist()
                sensor["width"] = width
                sensor["height"] = height
            else:
                # euler = cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["rotation"]
                # rotation = get_quaternion_from_euler(*euler, to_rad=True)
                intrinsic = []
            rotation = cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["quat"]
            # euler = cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["rotation"]
            # rotation = get_quaternion_from_euler(*euler, to_rad=True)
            calibration = {
                "token": calib_token,
                "sensor_token": sensor_token,
                "translation": cfg["SENSOR_CONFIG"][s]["TRANSFORM"]["location"],
                "rotation": rotation,
                "camera_intrinsic": intrinsic
            }
            sensors.append(sensor)
            calibrations.append(calibration)
            self.last_sample_data_tokens.append("")

        self.calibrated_sensors = calibrations
        self.sensors = sensors
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

    def save_ego_pose_data(self, filename, pose):
        x = -pose.location.x
        y = pose.location.y
        z = pose.location.z
        pitch = pose.rotation.pitch
        yaw = pose.rotation.yaw+180
        roll = pose.rotation.roll
        ego_pose = {
            "token": uuid1().hex,
            "translation": [x, y, z],
            "rotation": get_quaternion_from_euler(pitch, yaw, roll, to_rad=True),
            "timestamp": self.timestamp
        }
        self.ego_pose = ego_pose
        append_json(filename, ego_pose)

    def save_training_files(self, data):

        lidar_fname = self.LIDAR_PATH.format(self.captured_frame_id)
        # kitti_label_fname = self.KITTI_LABEL_PATH.format(self.captured_frame_id)
        # carla_label_fname = self.CARLA_LABEL_PATH.format(self.captured_frame_id)
        # img_fname = self.IMAGE_PATH.format(self.captured_frame_id)
        # calib_filename = self.CALIBRATION_PATH.format(self.captured_frame_id)
        can_bus_fname = self.CAN_BUS_PATH.format(self.scene_id)

        for agent, dt in data["agents_data"].items():
            # camera_transform= config_to_trans(self.cfg["SENSOR_CONFIG"]["RGB"]["TRANSFORM"])
            # lidar_transform = config_to_trans(self.cfg["SENSOR_CONFIG"]["LIDAR"]["TRANSFORM"])
            save_can_bus_data(can_bus_fname, dt["pose"], dt["imu"])
            self.save_ego_pose_data(self.EGO_POSE_PATH, dt["pose"])
            save_ref_files(self.OUTPUT_FOLDER, self.captured_frame_id)
            save_image_data(self.IMAGE_PATH, dt["sensor_data"][1:7], self.captured_frame_id)
            self.save_sample_data(dt)
            # save_label_data(kitti_label_fname, dt["kitti_datapoints"])
            # save_label_data(carla_label_fname, dt['carla_datapoints'])
            self.post_proc_sample_annotation(dt['nuscenes_datapoints'])
            self.save_instance()
            self.save_sample_annotation(self.SAMPLE_ANNO_PATH)
            # save_calibration_matrices([camera_transform, lidar_transform], calib_filename, dt["intrinsic"])
            save_lidar_data(lidar_fname, dt["sensor_data"][0])
        self.captured_frame_id += 1

    def save_sample_annotation(self, filename):
        anno_jsons = []
        for anno in self.annos:
            anno_json = anno.to_json()
            anno_jsons.append(anno_json)
        extend_json(filename, anno_jsons)

    def post_proc_sample_annotation(self, annos):
        # traverse the annotation for a sample and update instance & sample info
        for anno in annos:
            anno.set_sample_token(self.sample_token)
            if str(anno.carla_id) not in self.instances:
                instance = {
                    "carla_id": anno.carla_id,
                    "token": uuid1().hex,
                    "category_token": self.cfg["ANNOTATE_CATEGORIES"][anno.category],
                    "nbr_annotations": 0,
                    "first_annotation_token": anno.token,
                    "last_annotation_token": anno.token
                }
                self.instances[str(instance["carla_id"])] = instance
            else:
                # TODO optimze performance
                for prev_anno in self.annos:
                    if prev_anno.token == self.instances[str(anno.carla_id)]["last_annotation_token"]:
                        prev_anno.set_next(anno.token)
                        anno.set_prev(prev_anno.token)
                        break
                self.instances[str(anno.carla_id)]["last_annotation_token"] = anno.token
                instance = self.instances[str(anno.carla_id)]
            anno.set_instance_token(instance["token"])
            self.annos.append(anno)
    
    def init_scene(self):
        print("scene: {}".format(self.scene_id))
        self.sample_id = 0
        self.prev_sample_token = ""
        self.sample_token = uuid1().hex
        self.samples = []
        self.next_sample_token = uuid1().hex
        self.instances = {}
        self.annos = []

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
        self.scene["nbr_samples"] = self.sample_id
        self.scene["last_sample_token"] = self.prev_sample_token
        extend_json(self.SAMPLE_PATH, self.samples)
        append_json(self.SCENE_PATH, self.scene)
    
    def init_sample(self):
        # TODO decouple init sample
        pass

    def save_sample(self):
        sample = {
            "token": self.sample_token,
            "timestamp": self.timestamp,
            "scene_token": self.scene["token"],
            "next": "",
            "prev": self.prev_sample_token
        }

        # if self.sample_id == 0:
        #     self.sample_token = self.next_sample_token
        #     self.next_sample_token = uuid1().hex
        # else:

        for prev_sample in self.samples:
            if prev_sample["token"] == self.prev_sample_token:
                prev_sample["next"] = self.sample_token

        self.prev_sample_token = self.sample_token
        self.sample_token = self.next_sample_token
        self.next_sample_token = uuid1().hex

        self.samples.append(sample)
        self.sample_id += 1
        print("sample: {}".format(self.sample_id))

    def save_sample_data(self, data):
        sample_datas = []
        # TODO save sensor data here
        for i, sensor in enumerate(self.sensors):
            if sensor["modality"] == "lidar":
                filename = "velodyne/{0:06}.bin".format(self.captured_frame_id)
                fileformat = "pcd"
                width = 0
                height = 0
            elif sensor["modality"] == "camera":
                filename = "image/{0}/{1:06}.png".format(sensor["channel"], self.captured_frame_id)
                fileformat = "jpg"
                width = sensor["width"]
                height = sensor["height"]
            prev_sample_data_token = self.last_sample_data_tokens[i]
            sample_data = {
                "token": uuid1().hex,
                "sample_token": self.sample_token,
                "ego_pose_token": self.ego_pose["token"],
                "calibrated_sensor_token": sensor["calib_token"],
                "filename": filename,
                "fileformat": fileformat,
                "width": width,
                "height": height,
                "timestamp": self.timestamp,
                "is_key_frame": True,
                "next": "",
                "prev": prev_sample_data_token
            }
            sample_datas.append(sample_data)
            if prev_sample_data_token != "":
                for prev_sample_data in sample_datas:
                    if prev_sample_data["token"] == prev_sample_data_token:
                        prev_sample_data["next"] = sample_data["token"]
                        break
        extend_json(self.SAMPLE_DATA_PATH, sample_datas)

    def save_instance(self):
        instances = []
        for k, v in self.instances.items():
            instances.append(v)
        extend_json(self.INSTANCE_PATH, instances)