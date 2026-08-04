import pybullet as p
import pybullet_data
import math
import time
import pdb
import numpy as np
import sys
import os


def load_poses_csv(path, delimiter=",", skip_header=0, dtype=float):
    M = np.loadtxt(path, delimiter=delimiter, skiprows=skip_header, dtype=dtype)
    k = M.shape[0] // 4
    return M.reshape(k, 4, 4)


def main(project_aria_file, simulation_gripper_file):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    start_position = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    panda_id = p.loadURDF(
        "franka_panda/panda.urdf", start_position, start_orientation, useFixedBase=True
    )
    # Load a plane
    plane_id = p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.4,
        cameraYaw=60,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0.35, 0.24],
    )

    # pdb.set_trace()
    # replace it with the joint configurations and gripper conditions

    arm_trajectory = np.loadtxt(project_aria_file, delimiter=",")
    gripper_condition = np.loadtxt(simulation_gripper_file, delimiter=",")

    gripper_open_position = [0.08, 0.08]  # both fingers open
    gripper_closed_position = [0.01, 0.01]
    arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]
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
    print(len(arm_trajectory))
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
            time.sleep(1.0 / 300)
    # pdb.set_trace()
    p.disconnect()
    time.sleep(0.5)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
