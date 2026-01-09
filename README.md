# Tello MPC - Model Predictive Control for DJI Tello Drone

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.8+-brightgreen)
![License](https://img.shields.io/badge/License-MIT-green)

A **Model Predictive Control (MPC)** framework for autonomous drone control with real-time visual target tracking. This project implements multiple MPC backends (CasADi + IPOPT, CasADi + FATROP, and ACADOS) for trajectory optimization and integrates YOLOv8/Roboflow for visual object detection.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [MPC Implementations](#mpc-implementations)
- [Visual Detection](#visual-detection)
- [Configuration](#configuration)
- [ROS2 Topics](#ros2-topics)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

This project provides a complete pipeline for autonomous drone control using Model Predictive Control (MPC). The system:

1. **Detects targets** in real-time using YOLO-based object detection
2. **Tracks targets** by computing optimal control commands via MPC
3. **Avoids obstacles** using Control Barrier Functions (CBFs) 
4. **Controls the drone** by publishing velocity commands to ROS2

The framework is designed for simulation with Gazebo and can be adapted for real DJI Tello drones.

---

## Features

| Feature | Description |
|---------|-------------|
| 🎯 **Visual Servoing** | Image-based target tracking using 2D-to-3D coordinate transformation |
| 🧮 **Multiple MPC Backends** | CasADi (IPOPT, FATROP) and ACADOS solvers |
| 🚧 **Obstacle Avoidance** | Control Barrier Function (CBF) constraints |
| 📷 **Real-time Detection** | YOLOv8 and Roboflow inference integration |
| 🔄 **ROS2 Integration** | Full ROS2 Humble support with QoS profiles |
| ⚡ **High-frequency Control** | Up to 100Hz control loop execution |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Tello MPC Control Pipeline                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐    ┌──────────────────┐    ┌───────────────────┐     │
│  │   Camera     │───▶│  YOLO Detection  │───▶│  2D → 3D          │     │
│  │   /image_raw │    │  (YOLOv8/Roboflow)│    │  Coordinate       │     │
│  └──────────────┘    └──────────────────┘    │  Transform        │     │
│                                               └─────────┬─────────┘     │
│                                                         │               │
│  ┌──────────────┐                                       ▼               │
│  │   Odometry   │───────────────────────────────▶┌─────────────────┐   │
│  │   /odom      │                                │   MPC Solver    │   │
│  └──────────────┘                                │  ┌───────────┐  │   │
│                                                  │  │ CasADi    │  │   │
│                                                  │  │ - IPOPT   │  │   │
│                                                  │  │ - FATROP  │  │   │
│                                                  │  ├───────────┤  │   │
│                                                  │  │ ACADOS    │  │   │
│                                                  │  │ - SQP_RTI │  │   │
│                                                  │  └───────────┘  │   │
│                                                  └────────┬────────┘   │
│                                                           │             │
│                                                           ▼             │
│                                              ┌───────────────────────┐ │
│                                              │  Twist Publisher      │ │
│                                              │  /drone1/cmd_vel      │ │
│                                              │  (linear.x, z, ang.z) │ │
│                                              └───────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
Tello_MPC/
├── acados/                           # ACADOS-based MPC implementations
│   ├── acados.py                     # Basic ACADOS example
│   ├── acados_drone_interface.py     # Full drone interface with detection
│   ├── acados_external.py            # External cost MPC with CBF constraints
│   ├── acados_external_copy.py       # Extended version with obstacle avoidance
│   ├── acados_linear.py              # Linear MPC formulation
│   └── detect.py                     # Detection wrapper for ACADOS
│
├── casadi/                           # CasADi-based MPC implementations
│   ├── drone_interface.py            # ROS2 drone interface node
│   ├── mpc.py                        # Full MPC with FATROP/IPOPT solver
│   ├── mpc_ipopt.py                  # IPOPT-specific implementation
│   ├── mpc_modular.py                # Modular MPC class structure
│   └── mpc_modular_copy.py           # Extended modular implementation
│
├── utils/                            # Utility scripts
│   ├── cylinder.py                   # Obstacle representation utilities
│   ├── image.py                      # Image processing utilities
│   ├── image_save.py                 # Image saving and logging
│   ├── pred.py                       # Prediction utilities
│   ├── sub.py                        # ROS2 subscriber utilities
│   ├── transform.py                  # Coordinate transformation utilities
│   └── vel_pub.py                    # Manual velocity publisher
│
├── yolo/                             # YOLO detection module
│   ├── best.pt                       # YOLOv8 trained weights
│   ├── detect.py                     # Detection script
│   ├── load_model.py                 # YOLOv8/Ultralytics model loader
│   └── pred.py                       # Prediction wrapper
│
└── README.md                         # This file
```

---

## Prerequisites

### Hardware
- **GPU**: NVIDIA GPU recommended for real-time detection
- **Drone**: DJI Tello (simulation or real)

### Software
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.8+
- **ACADOS**: Latest version (for ACADOS backend)
- **CasADi**: 3.6+ (with FATROP or IPOPT)

---

## Installation

### 1. Install ROS2 Humble
```bash
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. Install Python Dependencies
```bash
pip install casadi numpy scipy matplotlib opencv-python
pip install ultralytics  # For YOLOv8
pip install inference    # For Roboflow
```

### 3. Install ACADOS (for ACADOS backend)
```bash
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build && cd build
cmake .. -DACADOS_WITH_QPOASES=ON
make install -j4

# Set environment variable
export ACADOS_SOURCE_DIR=/path/to/acados
```

### 4. Install CasADi with FATROP (optional, for faster solving)
```bash
pip install casadi
# FATROP requires additional build steps - see CasADi documentation
```

### 5. Clone and Build
```bash
cd ~/ros2_ws/src
git clone <repository-url> Tello_MPC
cd ~/ros2_ws
colcon build --packages-select tello_mpc
source install/setup.bash
```

---

## Usage

### Running the ACADOS-based Controller

```bash
# Terminal 1: Start your drone simulation or real drone
ros2 launch tello_gazebo tello_gazebo.launch.py

# Terminal 2: Run the MPC controller with detection
cd Tello_MPC/acados
python3 acados_drone_interface.py
```

### Running the CasADi-based Controller

```bash
# Terminal 1: Start simulation
ros2 launch tello_gazebo tello_gazebo.launch.py

# Terminal 2: Run CasADi MPC
cd Tello_MPC/casadi
python3 mpc.py
```

### Manual Velocity Control (Testing)

```bash
cd Tello_MPC/utils
python3 vel_pub.py
```

---

## MPC Implementations

### CasADi + IPOPT (`casadi/mpc_ipopt.py`)

**Description**: Nonlinear MPC using CasADi with IPOPT solver.

**Dynamics Model** (Unicycle):
```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω
```

**Parameters**:
| Parameter | Value | Description |
|-----------|-------|-------------|
| `N` | 100 | Prediction horizon |
| `step_horizon` | 0.082s | Time step |
| `Q` | diag(1, 5, 0.1) | State cost matrix |
| `R` | diag(0.5, 0.05) | Control cost matrix |
| `v_max` | 1 m/s | Max linear velocity |
| `ω_max` | π/4 rad/s | Max angular velocity |

### CasADi + FATROP (`casadi/mpc.py`)

**Description**: Faster solver using FATROP's structure detection.

```python
solver = casadi.nlpsol('solver', 'fatrop', nlp_prob, {
    "structure_detection": "auto",
    "debug": True,
    "equality": [True for _ in range(N * x.numel())]
})
```

### ACADOS SQP-RTI (`acados/acados_external.py`)

**Description**: Real-time iteration scheme for fast MPC with external cost.

**Dynamics Model** (3D Quadrotor):
```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
ż = w
θ̇ = ω
```

**Parameters**:
| Parameter | Value | Description |
|-----------|-------|-------------|
| `N` | 40 | Prediction horizon |
| `step_horizon` | 0.02s | Time step |
| `Q` | diag(1, 1, 0.2) | State cost matrix |
| `R` | diag(0.5, 0.7, 0.05) | Control cost matrix |

**Solver Settings**:
```python
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
```

---

## Visual Detection

### YOLOv8 (Local)

Using Ultralytics YOLOv8:
```python
from ultralytics import YOLO

model = YOLO('/path/to/best.pt')
results = model(frame)
```

### Roboflow API

Cloud-based detection:
```python
from inference import get_model

model = get_model(model_id="gazebo_mpc/1", api_key="YOUR_API_KEY")
model.confidence = 50
model.overlap = 25
results = model.infer(cv_image)
```

### 2D to 3D Coordinate Transform

```python
def convert_2D_3D(point, depth, fx=922, fy=922, cx=480, cy=360):
    x_d, y_d = point
    x_3d = (x_d - cx) * depth / fx
    y_3d = (y_d - cy) * depth / fy
    z_3d = depth
    return np.array([z_3d, -x_3d, -y_3d])
```

---

## Configuration

### Camera Intrinsics
```python
fx_d = 922      # Focal length x
fy_d = 922      # Focal length y  
cx_d = 480      # Principal point x
cy_d = 360      # Principal point y
```

### Control Limits
```python
# ACADOS
ocp.constraints.lbu = np.array([-4, -1, -np.pi*3.14/8])  # [v_min, w_min, ω_min]
ocp.constraints.ubu = np.array([4, 1, np.pi*3.14/8])     # [v_max, w_max, ω_max]

# Velocity scaling for Tello
twist.linear.x = u[0] / 8
twist.linear.z = u[1] / 8
twist.angular.z = u[2] / 3.14
```

### Obstacle Parameters (CBF)
```python
rho = 0.3       # Robot radius
ds = 1.2        # Safety distance
gamma = 0.3     # CBF decay rate
```

---

## ROS2 Topics

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/drone1/odom` | `nav_msgs/Odometry` | Drone odometry |
| `/drone1/image_raw` | `sensor_msgs/Image` | Camera feed |
| `/odom` | `nav_msgs/Odometry` | Target object odometry |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/drone1/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

### QoS Profile
```python
QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
```

---

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| **ACADOS not found** | Set `export ACADOS_SOURCE_DIR=/path/to/acados` |
| **CasADi FATROP error** | Fall back to IPOPT: `solver = casadi.nlpsol('solver', 'ipopt', nlp_prob, opts)` |
| **Drone not responding** | Check if `/drone1/cmd_vel` topic exists: `ros2 topic list` |
| **Detection slow** | Use local YOLOv8 instead of Roboflow API |
| **MPC infeasible** | Relax constraints or check initial state |

### Debug Commands

```bash
# Check topic publishing
ros2 topic echo /drone1/cmd_vel

# Monitor odometry
ros2 topic echo /drone1/odom

# Visualize camera
ros2 run rqt_image_view rqt_image_view /drone1/image_raw
```

---

## Performance Comparison

| Backend | Solve Time | Best For |
|---------|------------|----------|
| ACADOS SQP-RTI | ~5-10ms | Real-time control |
| CasADi + FATROP | ~20-50ms | Structure-exploiting problems |
| CasADi + IPOPT | ~50-200ms | General NLP, debugging |

---

## References

- [ACADOS Documentation](https://docs.acados.org/)
- [CasADi Documentation](https://web.casadi.org/docs/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [ROS2 Humble](https://docs.ros.org/en/humble/)

---

## License

This project is licensed under the MIT License.

---

## Acknowledgments

- [ACADOS Team](https://github.com/acados/acados) - Real-time optimal control
- [CasADi Developers](https://github.com/casadi/casadi) - Symbolic framework
- [Ultralytics](https://github.com/ultralytics/ultralytics) - YOLOv8
