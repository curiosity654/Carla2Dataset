from DataSave import DataSave
from SynchronyModel import SynchronyModel
from config import cfg_from_yaml_file
from data_utils import objects_filter

def main():
    cfg = cfg_from_yaml_file("configs_bev.yaml")
    model = SynchronyModel(cfg)
    dtsave = DataSave(cfg)
    try:
        model.set_synchrony()
        model.spawn_actors()
        model.set_actors_route()
        spawn_agent_suc = model.spawn_agent()
        model.sensor_listen()
        dtsave.init_scene()

        step = 0
        STEP = cfg["SAVE_CONFIG"]["STEP"]
        SAMPLE_PER_SCENE = cfg["SAVE_CONFIG"]["SAMPLE_PER_SCENE"]
        SCENE_NUM = cfg["SAVE_CONFIG"]["SCENE_NUM"]
        while True:
            if step % STEP == 0:
                # TODO add tokens
                data = model.tick()
                dtsave.save_sample(data["timestamp"])
                data = objects_filter(data)
                dtsave.save_training_files(data)
                if dtsave.sample_id % SAMPLE_PER_SCENE == 0 and dtsave.sample_id > 0:
                    dtsave.save_scene()
                    dtsave.init_scene()
                    if dtsave.scene_id == SCENE_NUM:
                        break
            else:
                model.world.tick()
            step += 1
    finally:
        model.setting_recover()


if __name__ == '__main__':
    main()
