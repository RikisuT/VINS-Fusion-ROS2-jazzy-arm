

# VINS-Fusion for ROS2 Jazzy on ARM

This repository provides a working implementation of **VINS-Fusion for ROS2 Jazzy**, specifically configured for ARM-based systems using an Intel RealSense D435i camera. It has been successfully ported to use **Ceres Solver 2.2.0**.

This is a fork of [zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2). The purpose of this fork is to provide a clear, step-by-step guide on how to get the system running.

-----

## Prerequisites

  * **Hardware**: An ARM-based computer (e.g., NVIDIA Jetson series) with an Intel RealSense D435i depth camera.
  * **Software**:
      * Ubuntu 24.04
      * ROS2 Jazzy Jubilee
      * Docker (for optional calibration)

-----

## Installation

This section covers the universal installation steps required for all use cases.

This guide assumes you have a ROS2 Jazzy workspace. If not, create one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 1\. Install System Dependencies

This project supports **Ceres Solver 2.2.0**, which is available directly from the Ubuntu 24.04 repositories.

  * Install Ceres Solver and its related dependencies using `apt`:
    ```bash
    sudo apt update
    sudo apt install libceres-dev
    ```
    This command also automatically installs required libraries like Eigen3.

### 2\. Build `realsense-ros` from Source

The stock `realsense-ros` package requires a code modification to its QoS settings to ensure stable performance.

  * Clone the `realsense-ros` repository using the **`ros2-master`** branch.

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
    ```

  * **Modify QoS Settings**
    Open `realsense-ros/realsense2_camera/src/rs_node_setup.cpp`. Locate the section where the `_synced_imu_publisher` is created and modify the code to enforce a `RELIABLE` QoS policy. This change is crucial for preventing dropped messages. For more details, see the discussion in [realsense-ros GitHub Issue \#3033](https://github.com/IntelRealSense/realsense-ros/issues/3033).

    **Change this block:**

    ```cpp
    {
        rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(HID_QOS);

        _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("~/imu",
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)));
    }
    ```

    **To this:**

    ```cpp
    {
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
       _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("~/imu",
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)));
    }
    ```

  * Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select realsense2_camera
    source install/setup.bash
    ```

### 3\. Build VINS-Fusion and Supporting Packages

  * Clone this repository into your `src` directory:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/RikisuT/VINS-Fusion-ROS2-jazzy-arm.git
    ```
  * Install remaining ROS dependencies and build all packages in the repository.
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    source install/setup.bash
    ```
    This will build `vins_fusion`, `camera_models`, `global_fusion`, and `loop_fusion`.

-----

## Quick Start: Running VINS-Fusion

After installation, you can run the system immediately. **If you just want to run the code, follow the steps below and skip the optional calibration section.**

### A) Running with a RealSense D435i Camera

1.  **Source Workspace**: Open a new terminal and source your workspace.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
2.  **Launch RealSense Camera**: In the same terminal, launch the camera using the provided configuration file.
    ```bash
    ros2 launch realsense2_camera rs_launch.py config_file:=./src/VINS-Fusion-ROS2-jazzy-arm/config/my_config/realsense_d435i_camera_config.yaml
    ```
3.  **Link TF Frames**: Open a *second* terminal, source your workspace, and publish a static transform between the `map` and `world` frames for visualization.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "world"
    ```
4.  **Launch VINS-Fusion**: Open a *third* terminal, source your workspace, and launch the main VINS node with its configuration file.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run vins vins_node ./src/VINS-Fusion-ROS2-jazzy-arm/config/my_config/d435i_final.yaml
    ```

### B) Running with the EuRoC Dataset

1.  **Download Dataset**: Download a ROS1 bag file from the [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) page (e.g., `MH_01_easy.bag`).

2.  **Convert Bag to ROS2**: The downloaded bags are in ROS1 format. Convert them to ROS2 using the `rosbags-convert` tool.

      * Install the tool if you haven't already:
        ```bash
        pip install rosbags
        ```
      * Run the conversion:
        ```bash
        # Replace with your downloaded file name
        rosbags-convert --src MH_01_easy.bag --dst MH_01_easy_ros2
        ```

3.  **Run the System**: This requires three separate terminals.

      * **Terminal 1: Play the Bag**
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 bag play MH_01_easy_ros2/
        ```
      * **Terminal 2: Link TF Frames**
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "world"
        ```
      * **Terminal 3: Launch VINS-Fusion**
        ```bash
        source ~/ros2_ws/install/setup.bash
        ros2 run vins vins_node ./src/VINS-Fusion-ROS2-jazzy-arm/config/euroc/euroc_stereo_imu_config.yaml
        ```

-----

## (Optional) Calibration for a Custom Camera

**This section is only for advanced users who want to re-calibrate for their own camera instead of using the provided configuration.**

### 1\. Record Calibration Data

  * Launch the RealSense node with the appropriate settings.
    ```bash
    ros2 launch realsense2_camera rs_launch.py config_file:=./src/VINS-Fusion-ROS2-jazzy-arm/config/my_config/realsense_d435i_camera_config.yaml
    ```
  * In another terminal, record a new bag file. Move the camera to view a calibration target from many different angles and distances.
    ```bash
    ros2 bag record /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu -o realsense_calib_bag
    ```

### 2\. Convert ROS2 Bag to ROS1

The Kalibr toolbox requires a ROS1 bag file.

```bash
pip install rosbags
rosbags-convert --src realsense_calib_bag/ --dst realsense_calib.bag
```

### 3\. Run Kalibr in Docker

Follow the [official Kalibr Docker instructions](https://www.google.com/search?q=https://github.com/ethz-asl/kalibr/wiki/installation%23docker) to run the container and perform the calibration using your converted `.bag` file.

-----

## Configuration

After running a custom calibration, you must update the VINS-Fusion configuration file with the new values from Kalibr.

1.  Open your configuration file (e.g., `config/my_config/d435i_final.yaml`).
2.  Manually transfer the parameters from the Kalibr output files. Pay close attention to matrix conventions, as you may need to invert the transformation matrix.