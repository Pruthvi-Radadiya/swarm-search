#!/usr/bin/env python3
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

OUT = Path.home() / "swarm-search" / "analysis" / "coverage.csv"
PERIOD = 5.0


class CoverageLogger(Node):
    def __init__(self):
        super().__init__("coverage_logger")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map = None
        self.create_subscription(OccupancyGrid, "/shared_map", self._on_map, qos)
        self.create_timer(PERIOD, self._tick)

        self._f = open(OUT, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["wall_time", "unknown", "free", "occupied", "known_frac"])
        self._f.flush()
        self.get_logger().info(f"Writing every {PERIOD}s to {OUT}")

    def _on_map(self, msg):
        self._map = msg

    def _tick(self):
        if self._map is None:
            self.get_logger().warn("waiting for /shared_map...")
            return
        data = self._map.data
        n = len(data)
        unknown = sum(1 for v in data if v < 0)
        free = sum(1 for v in data if v == 0)
        occupied = sum(1 for v in data if v >= 100)
        known_frac = (free + occupied) / n if n else 0.0
        self._w.writerow(
            [f"{time.time():.3f}", unknown, free, occupied, f"{known_frac:.6f}"]
        )
        self._f.flush()
        self.get_logger().info(f"known_frac={known_frac:.3f}")


def main():
    rclpy.init()
    node = CoverageLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._f.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
