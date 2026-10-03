from rclpy.node import Node
import rclpy
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

MIN_CLUSTER_SIZE = 10


class FrontierDetector(Node):
    def __init__(self):
        super().__init__(
            "frontier_detector",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, value=True)
            ],
        )

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, "/shared_map", self._on_map, map_qos)
        self.frontiers_pub = self.create_publisher(PoseArray, "/frontiers", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/frontiers_markers", 10)
        self.get_logger().info("frontier detector: listening on /shared_map")

    def _on_map(self, msg: OccupancyGrid):

        cells = self._frontier_cells(msg)
        clusters = self._cluster_cells(cells)

        centroids = []
        for cluster in clusters:
            if len(cluster) < MIN_CLUSTER_SIZE:
                continue
            centroids.append(self._centroid_world(cluster, msg))

        self._publish_frontiers(centroids, msg.header.frame_id)
        self._publish_markers(centroids, msg.header.frame_id)

        self.get_logger().info(
            f"frontier_cells={len(cells)} clusters_kept={len(centroids)}",
            throttle_duration_sec=2.0,
        )

    def _frontier_cells(self, msg: OccupancyGrid):
        """Return list of (i, j) free cells next to unknown."""
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        cells = []

        def cell(i, j):
            if i < 0 or i >= width or j < 0 or j >= height:
                return None
            return data[j * width + i]

        for j in range(height):
            for i in range(width):
                if cell(i, j) != 0:
                    continue
                neighbors = (
                    cell(i - 1, j),
                    cell(i + 1, j),
                    cell(i, j - 1),
                    cell(i, j + 1),
                )
                if any(n == -1 for n in neighbors):
                    cells.append((i, j))

        return cells

    def _cluster_cells(self, cells):
        """BFS: group 4 connected frontier cells"""
        cells_set = set(cells)
        visited = set()
        clusters = []

        for start in cells:
            if start in visited:
                continue
            queue = [start]
            visited.add(start)
            cluster = []

            while queue:
                i, j = queue.pop()
                cluster.append((i, j))
                for ni, nj in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)):
                    nxt = (ni, nj)
                    if nxt not in visited and nxt in cells_set:
                        visited.add(nxt)
                        queue.append(nxt)

            clusters.append(cluster)
        return clusters

    def _centroid_world(self, cluster, msg: OccupancyGrid):
        """Grid (i,j) → meters in map frame (cell centers)."""
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        resolution = msg.info.resolution

        mi = sum(i for i, _ in cluster) / len(cluster)
        nj = sum(j for _, j in cluster) / len(cluster)
        x = ox + (mi + 0.5) * resolution
        y = oy + (nj + 0.5) * resolution
        return (x, y)

    def _publish_frontiers(self, centroids, frame_id):
        pa = PoseArray()
        pa.header = Header()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = frame_id
        for x, y in centroids:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.frontiers_pub.publish(pa)

    def _publish_markers(self, centroids, frame_id):
        ma = MarkerArray()
        # Delete old markers so RViz does not keep stale ones
        delete = Marker()
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        stamp = self.get_clock().now().to_msg()
        for idx, (x, y) in enumerate(centroids):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = "frontiers"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color.a = 0.75
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.0
            ma.markers.append(m)
        self.markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
