#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN

class ScanCluster(Node):
    def __init__(self):
        super().__init__('scan_cluster')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(MarkerArray, '/clusters', 10)
        self.get_logger().info("LiDAR Clustering Node Started!")

    def callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # Filter invalid data
        mask = np.isfinite(ranges)
        angles, ranges = angles[mask], ranges[mask]

        # Polar → Cartesian
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.vstack((xs, ys)).T

        if len(points) < 5:
            return

        # DBSCAN Clustering
        clustering = DBSCAN(eps=0.25, min_samples=5).fit(points)
        labels = clustering.labels_

        # Visualization
        marker_array = MarkerArray()
        for cluster_id in set(labels):
            if cluster_id == -1:
                continue
            
            cluster_points = points[labels == cluster_id]
            cx = np.mean(cluster_points[:, 0])
            cy = np.mean(cluster_points[:, 1])

            marker = Marker()
            marker.header.frame_id = "base_scan"
            marker.type = Marker.SPHERE
            marker.pose.position.x = float(cx)
            marker.pose.position.y = float(cy)
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = float(np.random.rand())
            marker.color.g = float(np.random.rand())
            marker.color.b = float(np.random.rand())
            marker.id = cluster_id
            marker_array.markers.append(marker)

        self.pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ScanCluster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
