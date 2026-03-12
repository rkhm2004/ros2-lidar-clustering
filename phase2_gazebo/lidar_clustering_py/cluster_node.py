#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_clustering_node')
        
        # Subscribe to the topic coming from the Bridge
        self.subscription = self.create_subscription(
            PointCloud2, 
            '/lidar/points', 
            self.listener_callback, 
            10
        )
        
        # Publisher for the Green Boxes
        self.marker_pub = self.create_publisher(MarkerArray, '/pc_clusters', 10)
        
        self.get_logger().info("Cluster Node Started Successfully")

    def listener_callback(self, msg):
        # 1. Read points (x, y, z) from the message
        # skip_nans=True removes 'NaN' but NOT 'inf' (infinity)
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # 2. robustly filter out 'inf' values to prevent DBSCAN crash
        clean_points = []
        for p in points_gen:
            # Check if x, y, and z are all finite numbers
            if np.isfinite(p[0]) and np.isfinite(p[1]) and np.isfinite(p[2]):
                clean_points.append([p[0], p[1], p[2]])

        if not clean_points:
            return

        # 3. Convert to NumPy Array with explicit float type
        # 'dtype=np.float32' fixes the "Expected 2D array, got 1D array" error
        points_np = np.array(clean_points, dtype=np.float32)

        # 4. Downsample (Optional but good for speed)
        # Takes every 5th point to make DBSCAN faster
        points_np = points_np[::5]

        # 5. Run DBSCAN Clustering
        # eps=0.7 meters distance, min_samples=10 points to form a cluster
        try:
            clustering = DBSCAN(eps=0.7, min_samples=10).fit(points_np)
        except Exception as e:
            self.get_logger().error(f"DBSCAN Error: {e}")
            return

        labels = clustering.labels_
        unique_labels = set(labels)
        
        # 6. Prepare Visualization Markers
        marker_array = MarkerArray()
        
        # Add a "DELETEALL" marker to clear old boxes from the previous frame
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        cluster_id = 0
        for label in unique_labels:
            if label == -1: 
                continue # Skip noise points

            # Extract points belonging to this specific cluster
            mask = (labels == label)
            cluster_points = points_np[mask]
            
            if len(cluster_points) == 0:
                continue
            
            # Calculate Bounding Box
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            center = (min_pt + max_pt) / 2
            size = max_pt - min_pt

            # Create the Cube Marker
            marker = Marker()
            marker.header = msg.header # Sync with the lidar frame
            marker.ns = "clusters"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            
            # Set size (ensure it's not 0)
            marker.scale.x = float(size[0]) if size[0] > 0.1 else 0.1
            marker.scale.y = float(size[1]) if size[1] > 0.1 else 0.1
            marker.scale.z = float(size[2]) if size[2] > 0.1 else 0.1
            
            # Color: Green, semi-transparent
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            cluster_id += 1

        # Publish the results to RViz
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
