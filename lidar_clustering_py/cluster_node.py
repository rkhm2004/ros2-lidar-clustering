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
        self.subscription = self.create_subscription(
            PointCloud2, '/lidar/points', self.listener_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pc_clusters', 10)
        self.get_logger().info("Cluster Node Started")

    def listener_callback(self, msg):
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points_gen)
        if not points_list: return

        points_np = np.array(points_list)[::5] # Downsample
        clustering = DBSCAN(eps=0.7, min_samples=10).fit(points_np)
        labels = clustering.labels_
        unique_labels = set(labels)
        
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        cluster_id = 0
        for label in unique_labels:
            if label == -1: continue
            mask = (labels == label)
            cluster_points = points_np[mask]
            
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            center = (min_pt + max_pt) / 2
            size = max_pt - min_pt

            marker = Marker()
            marker.header = msg.header
            marker.ns = "clusters"; marker.id = cluster_id
            marker.type = Marker.CUBE; marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.scale.x = float(size[0]) if size[0]>0.1 else 0.1
            marker.scale.y = float(size[1]) if size[1]>0.1 else 0.1
            marker.scale.z = float(size[2]) if size[2]>0.1 else 0.1
            marker.color.a = 0.5; marker.color.g = 1.0; marker.color.r = 0.0
            marker_array.markers.append(marker)
            cluster_id += 1

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
