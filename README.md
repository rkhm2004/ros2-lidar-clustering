# 🚀 ROS2 LiDAR Object Clustering (DBSCAN + RViz)

This project demonstrates **LiDAR-based object clustering** using **ROS2 Jazzy** and real-world automotive LiDAR data (`PointCloud2`). Using the **DBSCAN** (Density-Based Spatial Clustering of Applications with Noise) algorithm, raw points are grouped into distinct objects and visualized in **RViz2** as 3D markers.

---

## 📌 Project Overview
- **Dataset:** `lexus3-2024-04-05-gyor.mcap` (Ouster OS LiDAR)
- **Duration:** ~12.5 seconds
- **Primary Topic:** `/lexus3/os_center/points`
- **Fixed Frame:** `lexus3/os_center_a_laser_data_frame`
- **Key Features:**
    * PointCloud2 subscription and preprocessing.
    * Spatial clustering via Scikit-Learn (DBSCAN).
    * Real-time centroid calculation.
    * Visualization via `MarkerArray`.

---

## ✅ PROJECT STATUS

### ✔ Completed
- [x] ROS2 Jazzy workspace setup.
- [x] MCAP LiDAR bag playback and TF frame verification.
- [x] DBSCAN clustering implementation (Python).
- [x] Cluster centroid computation and visualization.
- [x] Basic RViz2 visualization setup.

### 🔄 In Progress / Enhancements
- [ ] **Bounding Boxes:** Generating 3D boxes around clusters.
- [ ] **Color Coding:** Assigning unique colors to different cluster IDs.
- [ ] **Performance:** Implementing KD-Tree for faster processing.

---

## 🛠️ REQUIREMENTS & INSTALLATION

### 1. System Environment
- **OS:** Ubuntu 24.04 (Noble) or 22.04 (Jammy)
- **ROS2 Version:** Jazzy Jalisco

### 2. Dependencies
Install the necessary system packages and Python libraries:

```bash
# Update system
sudo apt update

# Install ROS2 Jazzy Desktop and RViz2
sudo apt install -y ros-jazzy-desktop ros-jazzy-rviz2 libogre-1.12-dev

# Install Python dependencies
pip3 install numpy scikit-learn

📁 WORKSPACE SETUP & BUILD

Run these commands to initialize your workspace, create the package, and build the environment:
Bash

# Create and navigate to workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the ROS2 package
ros2 pkg create lidar_clustering_py \
    --build-type ament_python \
    --dependencies rclpy sensor_msgs geometry_msgs visualization_msgs

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

▶️ RUNNING THE PROJECT
Phase 1: Play LiDAR Data

Ensure your .mcap file is located in your Downloads folder (or update the path accordingly).
Bash

ros2 bag play ~/Downloads/lexus3-2024-04-05-gyor.mcap --clock -l

Phase 2: Launch Clustering Node

In a new terminal, source the workspace and run the node:
Bash

cd ~/ros2_ws
source install/setup.bash
ros2 run lidar_clustering_py pc_cluster

Phase 3: RViz2 Visualization

Launch rviz2 in a new terminal and apply the following settings:

    Global Options:

        Fixed Frame: lexus3/os_center_a_laser_data_frame

    Add PointCloud2:

        Topic: /lexus3/os_center/points

        Size: 0.03

    Add MarkerArray:

        Topic: /pc_clusters

🖼️ RESULTS

Once configured, you will observe the raw point cloud (red) overlaid with green spheres representing the calculated centroids of detected objects.
🧩 PROJECT STRUCTURE
Plaintext

ros2_ws/
├── src/
│   └── lidar_clustering_py/
│       ├── lidar_clustering_py/
│       │   ├── __init__.py
│       │   └── pc_cluster_node.py   # Main Clustering Logic
│       ├── package.xml
│       └── setup.py
├── docs/
│   └── screenshots/               # Visualization images
└── README.md
