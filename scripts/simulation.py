import math
import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
import cv2  # We use OpenCV to manually encode the video


def load_poses_csv(path, delimiter=",", skip_header=0, dtype=float):
    M = np.loadtxt(path, delimiter=delimiter, skiprows=skip_header, dtype=dtype)
    k = M.shape[0] // 4
    return M.reshape(k, 4, 4)


def main(arg1, arg2):
    project_aria_file = arg1
    simulation_gripper_file = arg2
    video_output_path = "simulation_output.mp4"

    # Connect in DIRECT mode (headless)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    start_position = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    panda_id = p.loadURDF(
        "franka_panda/panda.urdf", start_position, start_orientation, useFixedBase=True
    )
    plane_id = p.loadURDF("plane.urdf")

    # --- Setup Virtual Camera ---
    width = 640
    height = 480
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.5, 0.35, 0.24],
        distance=1.4,
        yaw=60,
        pitch=-30,
        roll=0,
        upAxisIndex=2,
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=width / height, nearVal=0.02, farVal=5
    )

    # --- Setup OpenCV Video Writer ---
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Codec for MP4
    out = cv2.VideoWriter(video_output_path, fourcc, 30.0, (width, height))

    arm_trajectory = np.loadtxt(project_aria_file, delimiter=",")
    gripper_condition = np.loadtxt(simulation_gripper_file, delimiter=",")

    gripper_open_position = [0.08, 0.08]
    gripper_closed_position = [0.01, 0.01]

    c = p.createConstraint(
        panda_id,
        9,
        panda_id,
        10,
        jointType=p.JOINT_GEAR,
        jointAxis=[1, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

    print(
        f"Simulating {len(arm_trajectory)} trajectory steps and rendering video... (this may take a moment)"
    )

    for i in range(len(arm_trajectory)):
        if gripper_condition[i] == 0:
            gripper_targets = gripper_open_position
        else:
            gripper_targets = gripper_closed_position

        for j in range(7):
            p.setJointMotorControl2(
                bodyIndex=panda_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=arm_trajectory[i, j],
            )

        p.setJointMotorControl2(
            panda_id, 9, p.POSITION_CONTROL, targetPosition=gripper_targets[0], force=50
        )
        p.setJointMotorControl2(
            panda_id,
            10,
            p.POSITION_CONTROL,
            targetPosition=gripper_targets[1],
            force=50,
        )

        for _ in range(2):
            p.stepSimulation()

        # Render and capture frame every 4th iteration (~30 fps equivalent for the video)
        if i % 4 == 0:
            # ER_TINY_RENDERER uses the CPU - completely safe for Docker!
            _, _, rgbImg, _, _ = p.getCameraImage(
                width=width,
                height=height,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=p.ER_TINY_RENDERER,
            )

            # Convert the raw data to an 8-bit unsigned integer array, then reshape
            frame = np.array(rgbImg, dtype=np.uint8).reshape((height, width, 4))
            frame_rgb = frame[:, :, :3]  # Drop Alpha channel
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)  # Convert to BGR
            out.write(frame_bgr)

    # Cleanup
    out.release()
    p.disconnect()

    abs_video_path = os.path.abspath(video_output_path)
    print("\n" + "=" * 60)
    print(" [SUCCESS] PyBullet Simulation Complete!")
    print(f" [VIDEO SAVED AT]: {abs_video_path}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
