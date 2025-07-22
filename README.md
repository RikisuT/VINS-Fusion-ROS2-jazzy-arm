# VINS-Fusion ROS2 Jazzy

## An optimization-based multi-sensor state estimator for ROS2 Jazzy on ARM devices

This repository contains a ROS2 Jazzy compatible version of VINS-Fusion, specifically tailored for ARM devices. It incorporates fixes like replacing `cv_bridge.h` with `cv_bridge.hpp` for better compatibility.

The core code is based on [zinuok's VINS-Fusion-ROS2 repository](https://github.com/zinuok/VINS-Fusion-ROS2), which itself is a ROS2 port of the original [VINS-Fusion by HKUST-Aerial-Robotics](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/vins_logo.png" width = 55% height = 55% div align=left />
<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.png" width = 34% height = 34% div align=center />

VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR). VINS-Fusion supports multiple visual-inertial sensor types (mono camera + IMU, stereo cameras + IMU, even stereo cameras only). A toy example of fusing VINS with GPS is also provided in the original project.

**Features:**
- Multiple sensor support (stereo cameras / mono camera+IMU / stereo cameras+IMU)
- Online spatial calibration (transformation between camera and IMU)
- Online temporal calibration (time offset between camera and IMU)
- Visual loop closure

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti_rank.png" width = 80% height = 80% />

The original VINS-Fusion was the **top** open-sourced stereo algorithm on [KITTI Odometry Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) (12.Jan.2019).

**Authors (Original VINS-Fusion):** [Tong Qin](http://www.qintonguav.com), Shaozu Cao, Jie Pan, [Peiliang Li](https://peiliangli.github.io/), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [Aerial Robotics Group](http://uav.ust.hk/), [HKUST](https://www.ust.hk/)

**Videos (Original VINS-Fusion):**

<a href="https://www.youtube.com/embed/1qye82aW7nI" target="_blank"><img src="http://img.youtube.com/vi/1qye82aW7nI/0.jpg"
alt="VINS" width="320" height="240" border="10" /></a>

**Related Paper:** (paper is not exactly same with code)

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), **best student paper award** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Shaojie Shen, IEEE Transactions on Robotics [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert)

*If you use VINS-Fusion for your academic research, please cite our related papers. [bib](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/paper_bib.txt)*

## 1. Prerequisites

### 1.1 **System**
- Ubuntu 20.04 (or later)
- ROS2 Jazzy

### 1.2. **Libraries**
- OpenCV & cv_bridge for ROS2 Jazzy
- Ceres Solver-2.1.0 (or compatible version)
- Eigen-3.3.9 (or compatible version)

### 1.3. **Calibration Tool**
For accurate visual-inertial odometry, it is highly recommended to calibrate your camera-IMU system using `Kalibr`. You can find the Kalibr project and installation instructions [here](https://github.com/ethz-asl/kalibr).

## 2. Build VINS-Fusion ROS2 Jazzy

1.  **Install dependencies:**
    Run the `install_external_deps.sh` script to install OpenCV, Ceres, and Eigen:
    ```bash
    ./install_external_deps.sh
    ```

2.  **Build the package:**
    Navigate to your ROS2 workspace source directory (e.g., `~/ros2_ws/src`) if not already there, clone this repository, then build using `colcon`:
    ```bash
    cd ~/ros2_ws/src
    git clone [https://github.com/RikisuT/VINS-Fusion-ROS2-jazzy-arm.git](https://github.com/RikisuT/VINS-Fusion-ROS2-jazzy-arm.git)
    cd ..
    colcon build --symlink-install
    source install/setup.bash
    ```
    *(Note: If you encounter build errors, ensure all prerequisites are met and your ROS2 environment is properly sourced.)*

## 3. Playing EuRoC Dataset Bags

To test VINS-Fusion with EuRoC datasets:

1.  **Download a sample EuRoC dataset bag:**
    ```bash
    ./get_example_data.sh
    ```

2.  **Convert the bag to ROS2 format:**
    If you don't have `rosbags-convert`, install it: `pip install rosbags`. Then convert the bag:
    ```bash
    rosbags-convert data/V1_02_medium.bag --dst /output/path/to/save/ros2_bag
    ```

3.  **Play the converted ROS2 bag:**
    ```bash
    ros2 bag play /output/path/to/save/ros2_bag/V1_02_medium
    ```

## 4. Launching VINS-Fusion ROS2 Jazzy

You can launch VINS-Fusion using `ros2 launch` or `ros2 run`. Remember to source your ROS2 workspace before running. My custom configuration files are located in `config/`.

### 4.1 Monocular Camera + IMU Example (EuRoC)

Open multiple terminals and run the following commands:

**Terminal 1 (Rviz):**
```bash
ros2 launch vins vins_rviz.launch.py
````

**Terminal 2 (VINS Node):**

```bash
ros2 run vins vins_node config/euroc/euroc_mono_imu_config.yaml
```

**Terminal 3 (Optional: Loop Fusion Node):**

```bash
ros2 run loop_fusion loop_fusion_node config/euroc/euroc_mono_imu_config.yaml
```

**Terminal 4 (Play EuRoC Bag):**

```bash
ros2 bag play YOUR_CONVERTED_EUROC_BAG_PATH/MH_01_easy
```

*(Green path is VIO odometry; red path is odometry under visual loop closure.)*

### 4.2 Stereo Cameras + IMU Example (EuRoC)

**Terminal 1 (Rviz):**

```bash
ros2 launch vins vins_rviz.launch.py
```

**Terminal 2 (VINS Node):**

```bash
ros2 run vins vins_node config/euroc/euroc_stereo_imu_config.yaml
```

**Terminal 3 (Optional: Loop Fusion Node):**

```bash
ros2 run loop_fusion loop_fusion_node config/euroc/euroc_stereo_imu_config.yaml
```

**Terminal 4 (Play EuRoC Bag):**

```bash
ros2 bag play YOUR_CONVERTED_EUROC_BAG_PATH/MH_01_easy
```

### 4.3 Stereo Cameras Only Example (EuRoC)

**Terminal 1 (Rviz):**

```bash
ros2 launch vins vins_rviz.launch.py
```

**Terminal 2 (VINS Node):**

```bash
ros2 run vins vins_node config/euroc/euroc_stereo_config.yaml
```

**Terminal 3 (Optional: Loop Fusion Node):**

```bash
ros2 run loop_fusion loop_fusion_node config/euroc/euroc_stereo_config.yaml
```

**Terminal 4 (Play EuRoC Bag):**

```bash
ros2 bag play YOUR_CONVERTED_EUROC_BAG_PATH/MH_01_easy
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/euroc.gif" width = 430 height = 240 />

## 5\. KITTI Example

### 5.1 KITTI Odometry (Stereo)

Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to `YOUR_DATASET_FOLDER`. For sequence 00:

**Terminal 1 (Rviz):**

```bash
ros2 launch vins vins_rviz.launch.py
```

**Terminal 2 (VINS KITTI Odometry Test):**

```bash
ros2 run vins kitti_odom_test config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/
```

*(Optional: `ros2 run loop_fusion loop_fusion_node config/kitti_odom/kitti_config00-02.yaml` for loop closure. The original project evaluated odometry on KITTI benchmark without loop closure.)*

### 5.2 KITTI GPS Fusion (Stereo + GPS)

Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to `YOUR_DATASET_FOLDER`. For `2011_10_03_drive_0027_synced`:

**Terminal 1 (Rviz):**

```bash
ros2 launch vins vins_rviz.launch.py
```

**Terminal 2 (VINS KITTI GPS Test):**

```bash
ros2 run vins kitti_gps_test config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/
```

**Terminal 3 (Global Fusion Node):**

```bash
ros2 run global_fusion global_fusion_node
```

*(Green path is VIO odometry; blue path is odometry under GPS global fusion.)*

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.gif" width = 430 height = 240 />

## 6\. VINS-Fusion on Car Demonstration

Download the [car bag](https://drive.google.com/open?id=10t9H1u8pMGDOI6Q2w2uezEq5Ib-Z8tLz) to `YOUR_DATASET_FOLDER`.

**Terminal 1 (Rviz):**

```bash
ros2 launch vins vins_rviz.launch.py
```

**Terminal 2 (VINS Node):**

```bash
ros2 run vins vins_node config/vi_car/vi_car.yaml
```

**Terminal 3 (Optional: Loop Fusion Node):**

```bash
ros2 run loop_fusion loop_fusion_node config/vi_car/vi_car.yaml
```

**Terminal 4 (Play Car Bag):**

```bash
ros2 bag play YOUR_DATASET_FOLDER/car.bag
```

*(Green path is VIO odometry; red path is odometry under visual loop closure.)*

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/car_gif.gif" width = 430 height = 240 />

## 7\. Run with Your Devices

VIO is not only a software algorithm, it heavily relies on hardware quality. For beginners, we recommend you to run VIO with professional equipment, which contains global shutter cameras and hardware synchronization.

### 7.1 Configuration File

Write a config file for your device. You can take config files of EuRoC and KITTI as examples, or use my provided custom config files in the `config/` directory.

### 7.2 Camera Calibration

VINS-Fusion supports several camera models (pinhole, mei, equidistant). You can use [camodocal](https://github.com/hengli/camodocal) to calibrate your cameras. Example data is provided under `/camera_models/calibrationdata`.

To calibrate using `camodocal` (after building it in your workspace):

```bash
cd ~/ros2_ws/src/VINS-Fusion-ROS2-jazzy-arm/camera_models/camera_calib_example/
ros2 run camera_models Calibrations -w 12 -h 8 -s 80 -i calibrationdata --camera-model pinhole
```

## 8\. Acknowledgements

We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 9\. License

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin \<qintonguavATgmail.com\>.

For commercial inquiries, please contact Shaojie Shen \<eeshaojieATust.hk\>.

```
```
