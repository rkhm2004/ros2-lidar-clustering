# ROS2 LiDAR Object Clustering (PointCloud2 + DBSCAN)

This project implements **LiDAR object clustering** using **ROS2 Jazzy**,  
real automotive **PointCloud2** data, and the **DBSCAN** clustering algorithm.  
Cluster centroids are visualized in **RViz2** using `MarkerArray`.

Dataset used:  
`lexus3-2024-04-05-gyor.mcap` (12.5 seconds)

---

# 📍 PROJECT STATUS — WHAT IS COMPLETED SO FAR 

As of now, the following parts of the project have been **successfully completed and verified**:

### ✔ Completed
- ROS2 workspace creation (`ros2_ws`)
- Python package creation (`lidar_clustering_py`)
- Dependency configuration (`rclpy`, `sensor_msgs`, `visualization_msgs`)
- Build and sourcing of workspace using `colcon`
- Real LiDAR `.mcap` dataset loading
- Continuous LiDAR bag playback using `ros2 bag`
- Verification of LiDAR topic `/lexus3/os_center/points`
- Verification of TF frames
- Implementation of LiDAR clustering using DBSCAN
- Execution of clustering node (`pc_cluster`)
- Publishing clustered object centroids as `MarkerArray` on `/pc_clusters`

### ⏳ In Progress / Next Steps
- RViz visualization tuning
- Adding bounding boxes
- Object tracking
- Final report and presentation

---

# 📍 PROJECT PHASES (PHASE 1–3 COMPLETED)

This section documents **all completed phases** with **exact commands**.

---

## ✅ PHASE 1 — Workspace & Package Setup (COMPLETED)

### ✔ Description
Initial ROS2 workspace and package setup.

---

## ✅ PHASE 2 — LiDAR Data Playback (COMPLETED)

### ✔ Description
Playback and verification of real LiDAR data using ROS2 bag.

---

## ✅ PHASE 3 — LiDAR Object Clustering Node (COMPLETED)

### ✔ Description
DBSCAN-based clustering of PointCloud2 data and publishing results.

---

## COMMANDS USED (PHASE 1 → PHASE 3)

```bash
# ==============================
# PHASE 1: Workspace & Package Setup
# ==============================

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create ROS2 Python package
cd src
ros2 pkg create lidar_clustering_py --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs visualization_msgs

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash


# ==============================
# PHASE 2: LiDAR Data Playback
# ==============================

# Check dataset information
ros2 bag info ~/Downloads/lexus3-2024-04-05-gyor.mcap

# Play LiDAR bag file in loop mode
ros2 bag play ~/Downloads/lexus3-2024-04-05-gyor.mcap --clock -l

# Verify LiDAR topic publishing
ros2 topic hz /lexus3/os_center/points

# Check available TF frames
ros2 topic echo /tf --once


# ==============================
# PHASE 3: LiDAR Object Clustering Node
# ==============================

# Open a new terminal, then run:
cd ~/ros2_ws
source install/setup.bash
ros2 run lidar_clustering_py pc_cluster
