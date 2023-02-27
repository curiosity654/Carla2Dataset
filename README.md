# Carla2Dataset
This project is based on [DataGenerator](https://github.com/mmmmaomao/DataGenerator), with modifications to support the nuscenes dataset format and the surround-view camera.



## Configuration
Configuration of the surround-view camera used in the nuscenes dataset is in `config_bev.yaml`, see the file for details, you can also refer to [Nuscenes](https://www.nuscenes.org/nuscenes#data-collection).

## Project Structure
1. The main function `generator.py` controls the overall data generation, sets the CARLA simulation to synchronous mode, saves all sensor data after simulating a certain number of steps (step in config), and generates annotation information.
2. The `SynchronyModel` class encapsulates some functions of the CARLA synchronous mode, such as controlling the generation of actors, agents, setting routes, and collecting sensor data.
3. The `DataSave` class encapsulates the tools for saving various data formats.
4. The `export_utils` module contains some dataset format conversion functions.
5. The `data_utils` module contains sensor data post-processing, including some basic projection functions, and `lidar_visible` generates 3d box annotations based on lidar data.
6. The `data_descriptor` contains the dataset annotation format to be generated, which can be extended in the future if other formats are needed.

## Usage
1. Configure the CARLA environment, it is recommended to download the Shipping version directly.
2. Start the Carla server, if you are using it on the server, it is recommended to enable off-screen rendering.
   ``` 
   ./CarlaUE4.sh -RenderOffScreen
   ```
3. Configure `config_bev.yaml` as needed.
4. ```
    python generator.py
   ```
5. After data generation, you can use `nuscenes_tutorial.ipynb` for visualization.

## Notes
1. The generated data does not contain attribute and map currently, so it cannot be parsed by the official api, you need to replace it with `nuscenes_dev.py` in this project.
2. CARLA uses the left-handed local coordinate system of UE, but the Nuscenes annotation uses the global coordinate system, so both point cloud data and annotations are converted before saving.