import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary


class RoomAviary(CtrlAviary):
    """CtrlAviary with a bounded rectangular room added as static walls."""

    def __init__(
        self, *args, room_size=3.0, wall_height=2.0, wall_thickness=0.1, **kwargs
    ):
        self.room_size = room_size
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        super().__init__(*args, **kwargs)

    def _addObstacles(self):
        def add_wall(half_extents, position):
            col = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half_extents, physicsClientId=self.CLIENT
            )
            vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                rgbaColor=[
                    0.7,
                    0.7,
                    0.75,
                    0.35,
                ],  # alpha lowered, walls are see-through now
                physicsClientId=self.CLIENT,
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=position,
                physicsClientId=self.CLIENT,
            )

        r, h, t = self.room_size, self.wall_height, self.wall_thickness
        add_wall([r / 2, t / 2, h / 2], [0, r / 2, h / 2])
        add_wall([r / 2, t / 2, h / 2], [0, -r / 2, h / 2])
        add_wall([t / 2, r / 2, h / 2], [r / 2, 0, h / 2])
        add_wall([t / 2, r / 2, h / 2], [-r / 2, 0, h / 2])
