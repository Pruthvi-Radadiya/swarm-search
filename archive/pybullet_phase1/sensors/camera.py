import numpy as np
import pybullet as p


def get_camera(drone_id, client, width=320, height=240, fov=90):
    pos, que = p.getBasePositionAndOrientation(drone_id, physicsClientId=client)

    rot_matrix = p.getMatrixFromQuaternion(que)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    forward_vector = [1, 0, 0]
    up_vector = rot_matrix.dot([0, 0, 1])
    cam_target = np.array(pos) + rot_matrix.dot(forward_vector)

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=pos, cameraTargetPosition=cam_target, cameraUpVector=up_vector
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=fov, aspect=float(width) / height, nearVal=0.06, farVal=10.0
    )

    img = p.getCameraImage(
        width,
        height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        physicsClientId=client,
    )

    return np.asarray(img[2], dtype=np.uint8).reshape(height, width, 4)
