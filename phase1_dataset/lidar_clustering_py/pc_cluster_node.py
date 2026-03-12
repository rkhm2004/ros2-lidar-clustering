#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import numpy as np
from sklearn.cluster import DBSCAN
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudCluster(Node):

    def __init__(self):
        super().__init__('pc_cluster')

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            '/lexus3/os_center/points',
            self.pointcloud_callback,
            10
        )

        # Publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pc_clusters',
            10
        )

        self.get_logger().info("PointCloud clustering node started")

    def pointcloud_callback(self, msg):
        self.get_logger().info("PointCloud received")

        points = []
        for p in pc2.read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if len(points) == 0:
            return

        points = np.array(points)

        # DBSCAN clustering
        clustering = DBSCAN(eps=1.0, min_samples=20).fit(points)
        labels = clustering.labels_

        marker_array = MarkerArray()
        marker_id = 0

        for label in set(labels):
            if label == -1:
                continue

            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "clusters"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {marker_id} clusters")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCluster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

