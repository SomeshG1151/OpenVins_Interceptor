# Visual-Inertial Odometry & 3D Feature Tracking Architecture

This document describes the complete architecture of the Real-Time Visual-Inertial Odometry (VIO) and persistent 3D feature tracking system built on **ROS 2 Humble**, **OpenVINS (`ov_msckf`)**, **ESP32 + MPU6050 IMU**, and a **monocular camera**.

The system estimates the 6-DoF ego-motion (position, attitude, velocity) of the sensor rig without GPS, and extracts metric 3D positions of persistent visual landmarks associated with unique feature IDs (`feature_id -> X, Y, Z`) in the global inertial frame.

---

## 1. High-Level System Architecture

```mermaid
flowchart TD
    subgraph Hardware ["1. Hardware Layer"]
        CAM_HW["Webcam (/dev/video0)<br>640x480 @ 30 FPS"] 
        IMU_HW["ESP32 + MPU6050 (/dev/ttyACM*)<br>6-DoF Accel + Gyro @ 200 Hz"]
    end

    subgraph Drivers ["2. ROS 2 Driver Layer"]
        V4L2["v4l2_camera_node<br>(V4L2 device streaming)"]
        IMU_DRV["esp32_imu_driver_node<br>(Auto-port detection + DTR reset)"]
        CAM_HW -->|Raw Video| V4L2
        IMU_HW -->|115200 Baud Serial| IMU_DRV
    end

    subgraph Middleware ["3. ROS 2 Transport"]
        V4L2 -->|/cam0/image_raw| Q_CAM["OpenVINS Ingestion Queue"]
        IMU_DRV -->|/imu/data_raw| Q_IMU["OpenVINS Ingestion Queue"]
    end

    subgraph Core ["4. OpenVINS State Estimator (ov_msckf)"]
        Q_IMU --> PROP["Kinematic Propagation<br>(RK4 Integration @ 200 Hz)"]
        Q_CAM --> FRONTEND["Visual Frontend<br>(FAST Corners + KLT Optical Flow)"]
        
        PROP --> EKF["Error-State Extended Kalman Filter (ES-EKF)"]
        FRONTEND --> MSCKF_UP["MSCKF Nullspace Update<br>(Sliding Window Clones)"]
        FRONTEND --> SLAM_UP["Persistent SLAM Update<br>(Anchored Inverse Depth)"]
        ZUPT_MOD["Zero Velocity Update (ZUPT)"] --> EKF

        MSCKF_UP --> EKF
        SLAM_UP --> EKF
        ZUPT_MOD --> EKF
    end

    subgraph OutputLayer ["5. Visualization & Data Extraction"]
        EKF -->|/ov_msckf/odometry| ODOM["3D Odometry (Position, Attitude, Velocity)"]
        EKF -->|/ov_msckf/path| PATH_VIZ["3D Trajectory Path in RViz2"]
        EKF -->|/ov_msckf/slam_features| FEAT_VIZ["Persistent Landmarks (feature_id -> X,Y,Z)"]
        
        FEAT_VIZ --> RVIZ["display_features_ros2.rviz<br>(3D Spheres + Floating ID Labels)"]
        FEAT_VIZ --> LOGGER["log_slam_features.py -> slam_features.csv"]
    end
```

---

## 2. Layer-by-Layer Architectural Breakdown

### 2.1 Hardware Layer

| Component | Interface / Port | Sampling Rate | Role |
| :--- | :--- | :--- | :--- |
| **Monocular Webcam** | `/dev/video0` (UVC) | 30 Hz | Captures 2D perspective bearing rays. Provides visual parallax as the camera translates. |
| **ESP32 + MPU6050** | `/dev/ttyACM*` (CDC-ACM USB) | 200 Hz | Measures specific force ($a - g$) and angular velocity ($\omega$). Senses Earth's gravity vector ($9.81\,\text{m/s}^2$) to establish the **metric scale** (meters). |
| **Mechanical Rigidity** | Rigid physical mounting | $\infty$ | The IMU and camera are rigidly coupled with zero flex. Their spatial offset is represented by the extrinsic transform $T_{cam}^{imu}$. |

---

### 2.2 Driver & Middleware Layer

#### A. `v4l2_camera` Node
* Streams native YUYV frames from `/dev/video0` at 640x480 resolution.
* Converts pixel encoding to standard RGB8 / Mono8 and attaches high-precision ROS timestamps.
* Publishes to `/cam0/image_raw`.

#### B. `esp32_imu_driver` Node
* **Automatic Port Detection:** Automatically searches candidate device paths (`/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyACM2`, `/dev/ttyUSB0`) so Linux serial re-enumeration on hot-reconnect never breaks startup.
* **Auto-Reset Circuit Management:** Pulses DTR and RTS lines (`TIOCMBIC` / `TIOCMBIS`) on initialization to clear the ESP32 hardware reset latch without getting stuck in the ROM bootloader.
* **POSIX Non-Blocking I/O:** Reads raw ASCII lines over serial at 115200 baud, parses 6-DoF accelerometer ($a_x, a_y, a_z$) and gyroscope ($\omega_x, \omega_y, \omega_z$) readings, and publishes `sensor_msgs/msg/Imu` to `/imu/data_raw`.

---

### 2.3 Synchronization & Queueing Mechanism

In Visual-Inertial Odometry, state propagation requires that IMU measurements strictly span the time between consecutive camera exposures.

In `ROS2Visualizer.cpp`:
1. Camera images enter a sorted queue (`camera_queue`).
2. An image is only popped and processed when the latest IMU timestamp is strictly ahead of the camera frame:
   $$\text{timestamp}_{imu} > \text{timestamp}_{cam} + t_{offset}$$
3. This guarantees causal state propagation and prevents Kalman filter divergence.

---

### 2.4 OpenVINS State Estimation Core (`ov_msckf`)

OpenVINS runs a **Type-Based Error-State Extended Kalman Filter (ES-EKF)**.

```mermaid
sequenceDiagram
    autonumber
    participant IMU as IMU Driver (200 Hz)
    participant EKF as OpenVINS EKF
    participant CAM as Camera Driver (30 Hz)

    IMU->>EKF: Linear Acceleration & Angular Velocity
    Note over EKF: 1. Predict: RK4 Kinematic Propagation<br>2. Propagate Error Covariance Matrix (Q)

    CAM->>EKF: Image Frame
    Note over EKF: 3. Visual Frontend: KLT Track Features<br>4. Augment State with Camera Clone Pose

    alt Sensor Stationary (ZUPT Active)
        Note over EKF: 5. Inject v = [0,0,0] measurement<br>Reset velocity drift & calibrate biases
    end

    Note over EKF: 6. Triangulate features across sliding window<br>7. MSCKF Nullspace Update (Remove feature states)
    Note over EKF: 8. Update Persistent SLAM Landmarks (Inverse Depth)<br>9. Marginalize oldest clone from window

    EKF-->>CAM: Updated Metric State (Position, Attitude, Speed)
```

#### 1. The Dynamic State Vector ($\mathbf{x}$)
$$\mathbf{x} = \begin{bmatrix} \mathbf{x}_{IMU} & \mathbf{x}_{clones} & \mathbf{x}_{SLAM} & \mathbf{x}_{calib} \end{bmatrix}^\top$$

* **Active IMU State ($\mathbf{x}_{IMU}$ - 15 Dimensions):**
  * Orientation: JPL Quaternion $^{G}_{I}\mathbf{q}$ (World-to-IMU rotation)
  * Position: $^{G}\mathbf{p}_{I}$ (3D coordinates in meters)
  * Linear Velocity: $^{G}\mathbf{v}_{I}$ (in $\text{m/s}$)
  * Gyroscope Bias: $\mathbf{b}_g$ (calibrated online)
  * Accelerometer Bias: $\mathbf{b}_a$ (calibrated online)
* **Sliding Window Clones ($\mathbf{x}_{clones}$ - $6 \times N$ Dimensions, $N=11$):**
  * Rolling history of camera poses used for multi-view feature triangulation.
* **Persistent SLAM Landmarks ($\mathbf{x}_{SLAM}$ - $3 \times M$ Dimensions, $M \le 50$):**
  * Visual landmarks maintained directly inside the Kalman state using **Anchored Inverse Depth** ($\rho = 1/d$).
* **Online Calibration Parameters ($\mathbf{x}_{calib}$):**
  * Spatial camera-to-IMU extrinsic transform ($T_C^I$) and time offset ($t_d$).

#### 2. Propagation Step (200 Hz)
* **Mean State:** Integrates non-linear kinematics using **Runge-Kutta 4th Order (RK4)** numerical integration.
* **Covariance Propagation:**
  $$\mathbf{P}_{k+1} = \boldsymbol{\Phi}_k \mathbf{P}_k \boldsymbol{\Phi}_k^\top + \mathbf{Q}_k$$
  where $\mathbf{Q}_k$ is derived from the sensor noise densities (`accelerometer_noise_density` and `gyroscope_noise_density` in `kalibr_imu_chain.yaml`).

#### 3. Measurement Update Step (30 Hz)
* **Visual Frontend:** FAST corner detection identifies salient texture points. KLT optical flow tracks points frame-to-frame.
* **MSCKF Nullspace Projection:** Features that reach the end of the clone window are triangulated. The measurement equation is projected onto the left nullspace of the feature Jacobian, updating all camera poses without adding feature coordinates into the state matrix (keeping computation $O(N^3)$ where $N$ is clone window size).
* **Persistent SLAM Promotion:** Long-tracked features are promoted into $\mathbf{x}_{SLAM}$. When re-observed, they provide direct EKF measurement updates that eliminate long-term drift.
* **Zero Velocity Update (ZUPT):** When visual disparity and IMU acceleration variance drop below stationary thresholds, OpenVINS injects a zero-velocity measurement ($\mathbf{v} = \mathbf{0}$), freezing position drift and re-centering sensor biases.

---

### 2.5 3D Feature Extraction & Visualization Layer

Standard OpenVINS publishes an anonymous point cloud (`/points_slam`) where individual feature identities are discarded. Our architecture exposes a persistent association:
$$\text{feature\_id} \longrightarrow (X, Y, Z)_{\text{global}}$$

```mermaid
graph LR
    subgraph OpenVINS Core
        STATE["State::_features_SLAM<br>std::unordered_map<size_t, Landmark>"]
    end

    subgraph Visualizer Layer
        CALC["Reconstruct Global 3D Pose:<br>p_FinG = R_GtoI^T * R_ItoC^T * (p_FinA - p_IinC) + p_IinG"]
        PUB["ROS2Visualizer::pub_slam_features"]
        TOPIC["/ov_msckf/slam_features<br>(visualization_msgs/MarkerArray)"]
    end

    subgraph Consumers
        RVIZ["RViz2 Display<br>Green Spheres + Yellow Floating IDs"]
        LOGGER["log_slam_features.py<br>Terminal Table + slam_features.csv"]
    end

    STATE --> CALC --> PUB --> TOPIC
    TOPIC --> RVIZ
    TOPIC --> LOGGER
```

#### Coordinate Transformation Math
For anchored features, coordinates are estimated relative to an anchor camera clone. The global position is reconstructed via:
$$p_{FinG} = R_{G \to I_{anchor}}^\top R_{I \to C_{anchor}}^\top \left( p_{FinAnchor} - p_{IinC_{anchor}} \right) + p_{IinG_{anchor}}$$

#### Output Message Structure
Topic: `/ov_msckf/slam_features` (`visualization_msgs/msg/MarkerArray`)
* **Sphere Markers (`ns="slam_features"`):** 3D position $(X, Y, Z)$ in meters, `id = feature_id`.
* **Text Markers (`ns="slam_features_id"`):** Floating 3D text showing the numeric ID above each landmark in RViz.

---

## 3. File & Repository Structure

```text
~/workspace/catkin_ws_ov/
├── ARCHITECTURE.md                  # Complete architectural documentation
├── log_slam_features.py             # Standalone Python script to log feature_id -> XYZ to CSV
│
├── src/
│   ├── esp32_imu_driver/            # Custom IMU Driver Package
│   │   ├── src/
│   │   │   └── esp32_imu_driver.cpp # Auto-port detection, DTR pulse, serial parser
│   │   └── launch/
│   │       ├── esp32_imu.launch.py            # Standalone IMU launch
│   │       ├── full_system.launch.py          # Original full VIO system launch (Untouched)
│   │       └── full_system_features.launch.py # Dedicated launch with Feature ID RViz display
│   │
│   └── open_vins/                   # OpenVINS Core State Estimator
│       ├── config/my_laptop_esp32/
│       │   ├── estimator_config.yaml          # Filter options, ZUPT, feature counts
│       │   ├── kalibr_imu_chain.yaml          # Realistic MPU6050 noise & random walk parameters
│       │   └── kalibr_imucam_chain.yaml       # Camera intrinsics and camera-IMU extrinsics
│       └── ov_msckf/
│           ├── launch/
│           │   ├── display_ros2.rviz          # Standard RViz display (Untouched)
│           │   └── display_features_ros2.rviz # RViz display with MarkerArray Feature IDs
│           └── src/ros/
│               ├── ROS2Visualizer.h           # Declares pub_slam_features
│               └── ROS2Visualizer.cpp         # Computes global XYZ and publishes MarkerArray
```

---

## 4. Operational Guide

### 4.1 Build
Always build single-threaded on resource-constrained systems:
```bash
cd ~/workspace/catkin_ws_ov
colcon build --packages-select ov_msckf esp32_imu_driver --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### 4.2 Run
#### Option A: Run System with Feature ID Visualization (New Setup)
```bash
source ~/workspace/catkin_ws_ov/install/setup.bash
ros2 launch esp32_imu_driver full_system_features.launch.py
```

#### Option B: Run Original System (Untouched Setup)
```bash
source ~/workspace/catkin_ws_ov/install/setup.bash
ros2 launch esp32_imu_driver full_system.launch.py
```

### 4.3 Log Feature IDs & 3D Coordinates to CSV
In a second terminal:
```bash
source ~/workspace/catkin_ws_ov/install/setup.bash
python3 ~/workspace/catkin_ws_ov/log_slam_features.py
```
Output format saved to `slam_features.csv`:
```text
Timestamp      | Feature ID | X (m)     | Y (m)     | Z (m)    
-----------------------------------------------------------------
179134.4210    | 102        | 0.4215    | -0.1732   | 2.8410   
179134.4210    | 117        | 0.8321    | -0.0924   | 3.1042   
```

### 4.4 Clean System Shutdown
To close cleanly:
```bash
# In the launch terminal:
Ctrl + C

# If any process hangs:
killall -9 run_subscribe_msckf v4l2_camera_node esp32_imu_driver rviz2 2>/dev/null
```

---

## 5. Future Drone Roadmap (Target 3D Localization)

This ego-motion pipeline forms the foundation for autonomous drone navigation and target interception:

```mermaid
flowchart LR
    CAM["Camera Feed (/cam0/image_raw)"] --> YOLO["Visual Target Detector (YOLO / ArUco)"]
    CAM --> VIO["OpenVINS (Ego-Motion Estimator)"]
    
    YOLO -->|2D Target Center (u, v)| ASSOC["3D Target Association Node"]
    VIO -->|Drone World Pose (X, Y, Z, Q)| ASSOC
    VIO -->|Persistent 3D Landmarks| ASSOC
    
    ASSOC -->|Target 3D Coordinates wrt Drone| GUIDANCE["Drone Interception / Tracking Controller"]
```

1. **Ego-Motion State:** OpenVINS provides the drone's position, tilt, and velocity relative to the launch pad.
2. **Target Detection:** An external perception node (YOLOv8 or ArUco detector) detects the target in pixel space $(u, v)$.
3. **Triangulation / Association:** By projecting the bearing ray from the drone's current pose and intersecting it with the persistent SLAM landmarks or depth estimation, the system computes:
   $$P_{\text{target}}^{\text{drone}} = \begin{bmatrix} X & Y & Z \end{bmatrix}^\top$$
4. **Guidance:** Enables real-time target tracking and autonomous drone interception without GPS.
