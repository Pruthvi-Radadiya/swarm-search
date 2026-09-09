import numpy as np
import pybullet as p


class Lidar:
    """A simulated 2D lidar ring for one drone."""

    def __init__(self, num_rays=36, max_range=3.0):
        self.num_rays = num_rays
        self.max_range = max_range
        self.line_ids = [None] * num_rays  # Store line IDs for visualization

    def sense(self, drone_id, client):
        pos, _ = p.getBasePositionAndOrientation(drone_id, physicsClientId=client)
        ray_froms = []
        ray_tos = []

        for i in range(self.num_rays):
            angle = (2 * np.pi / self.num_rays) * i
            dx = self.max_range * np.cos(angle)
            dy = self.max_range * np.sin(angle)
            ray_froms.append(pos)
            ray_tos.append([pos[0] + dx, pos[1] + dy, pos[2]])

        results = p.rayTestBatch(ray_froms, ray_tos, physicsClientId=client)

        distances = []
        hit_positions = []
        for result in results:
            hit_fraction = result[2]
            distance = hit_fraction * self.max_range
            distances.append(distance)
            hit_positions.append(result[3])

        self.scan_visualization(ray_froms, hit_positions, client)

        return distances

    def scan_visualization(self, ray_froms, hit_positions, client):
        for i in range(self.num_rays):
            if self.line_ids[i] is None:
                self.line_ids[i] = p.addUserDebugLine(
                    ray_froms[i],
                    hit_positions[i],
                    lineColorRGB=[1, 0, 0],
                    lineWidth=2.0,
                    physicsClientId=client,
                )
            else:
                p.addUserDebugLine(
                    ray_froms[i],
                    hit_positions[i],
                    lineColorRGB=[1, 0, 0],
                    lineWidth=2.0,
                    replaceItemUniqueId=self.line_ids[i],
                    physicsClientId=client,
                )
