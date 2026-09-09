import time

import cv2
import numpy as np
import pybullet as p
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from envs.room_env import RoomAviary
from sensors.camera import get_camera
from sensors.imu import IMU
from sensors.lidar import Lidar

env = RoomAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    initial_xyzs=np.array([[0, 0, 1]]),
    physics=Physics.PYB,
    pyb_freq=240,
    ctrl_freq=48,
    gui=True,
    record=False,
    obstacles=True,
    user_debug_gui=True,
)

# default camera: angled, looking down into the room from just outside it
p.resetDebugVisualizerCamera(
    cameraDistance=4,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 1],
    physicsClientId=env.CLIENT,
)

obs, info = env.reset()
hover = env.HOVER_RPM
drone_id = env.DRONE_IDS[0]

pos, _ = p.getBasePositionAndOrientation(drone_id, physicsClientId=env.CLIENT)

ghost_visual = p.createVisualShape(
    p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 0.6], physicsClientId=env.CLIENT
)
ghost_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=ghost_visual,
    basePosition=pos,
    physicsClientId=env.CLIENT,
)

lidar = Lidar(num_rays=36, max_range=3.0)
imu = IMU(initial_pos=pos)

for i in range(3000):
    step_start = time.time()
    action = np.array([[hover, hover, hover, hover]])
    obs, reward, terminated, truncated, info = env.step(action)

    pos, _ = p.getBasePositionAndOrientation(drone_id, physicsClientId=env.CLIENT)
    distances = lidar.sense(drone_id, client=env.CLIENT)

    rgba_image = get_camera(drone_id, env.CLIENT, width=320, height=240, fov=90)
    bgr_image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2BGR)
    cv2.imshow("Drone Camera", bgr_image)
    cv2.waitKey(1)

    believed_pos = imu.update(pos)
    p.resetBasePositionAndOrientation(
        ghost_id, believed_pos, [0, 0, 0, 1], physicsClientId=env.CLIENT
    )

    p.applyExternalForce(
        drone_id,
        -1,
        forceObj=[0.02, 0, 0],
        posObj=[0, 0, 0],
        flags=p.LINK_FRAME,
        physicsClientId=env.CLIENT,
    )

    elapsed = time.time() - step_start
    time.sleep(max(0, 1.0 / env.CTRL_FREQ - elapsed))
    if terminated or truncated:
        obs, info = env.reset()

env.close()
