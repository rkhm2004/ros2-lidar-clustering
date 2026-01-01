#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2 as pc2

import numpy as np
from sklearn.cluster import DBSCAN


class PCCluster(Node):
    def __init__(self):
        super().__init__('pc_cluster')

        # Subscribe to Lexus LiDAR pointcloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lexus3/os_center/points',
            self.pc_callback,
            10
        )

        # Publisher for cluster centroids
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pc_clusters',
            10
        )

        self.get_logger().info('PointCloud2 clustering node started')

    def pc_callback(self, msg: PointCloud2):
        points_xy = []

        # Read PointCloud2
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p

            # Simple ground filtering
            if -1.0 < z < 2.0:
                points_xy.append([x, y])

        if len(points_xy) < 30:
            return

        pts = np.array(points_xy)

        # DBSCAN clustering
        clustering = DBSCAN(
            eps=0.6,        # distance threshold (meters)
            min_samples=15  # minimum points per cluster
        ).fit(pts)

        labels = clustering.labels_
        unique_labels = set(labels)

        marker_array = MarkerArray()

        for label in unique_labels:
            if label == -1:
                continue  # noise

            cluster_pts = pts[labels == label]
            cx, cy = cluster_pts.mean(axis=0)

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'pc_clusters'
            marker.id = int(label)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(cx)
            marker.pose.position.y = float(cy)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 0.6

            # Color (deterministic)
            marker.color.r = (label * 37 % 255) / 255.0
            marker.color.g = (label * 97 % 255) / 255.0
            marker.color.b = (label * 57 % 255) / 255.0
            marker.color.a = 0.9

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PCCluster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

