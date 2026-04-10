import time
import numpy as np
import cv2
import pybullet as p
import pybullet_data
import os



def setup_scene():
    """
    Connect to PyBullet, load scene objects and robot, and return:
        (robot_id, object_ids)

    Caller is responsible for calling p.disconnect().
    """
    p.connect(p.GUI)
    time.sleep(0.5)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / 240.0)

    # Turn off extra visualizer panels for a cleaner GUI and faster setup
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # ------------------------------------------------------------------
    # Load environment
    # ------------------------------------------------------------------
    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf", basePosition=[0, 0, 0], useFixedBase=True)

    # ------------------------------------------------------------------
    # Load objects
    # ------------------------------------------------------------------
    cube1 = p.loadURDF("cube_small.urdf", basePosition=[0.4, 0.0, 0.65])
    cube2 = p.loadURDF("cube_small.urdf", basePosition=[0.2, 0.4, 0.65])
    cube3 = p.loadURDF("cube_small.urdf", basePosition=[-0.3, -0.1, 0.65])

    object_ids = [cube1, cube2, cube3]

    # Improve object friction
    for obj_id in object_ids:
        p.changeDynamics(
            obj_id, -1,
            lateralFriction=5.0,
            rollingFriction=0.002,
            spinningFriction=0.002,
            restitution=0.0
        )

    # ------------------------------------------------------------------

    # Load robot
    CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
    robot_urdf = os.path.join(CURRENT_DIR, "urdf", "ur5_robotiq_85.urdf")
    # ------------------------------------------------------------------
    robot_id = p.loadURDF(
        robot_urdf,
        basePosition=[0, -0.4, 0.62],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57]),
        useFixedBase=True
    )

    # Improve finger pad friction
    for pad_link in [12, 17]:
        p.changeDynamics(
            robot_id, pad_link,
            lateralFriction=5.0,
            rollingFriction=0.001,
            spinningFriction=0.001,
            restitution=0.0
        )

    # ------------------------------------------------------------------
    # Set arm to neutral pose
    # ------------------------------------------------------------------
    arm_joints = get_arm_joints(robot_id)
    start_positions = [0, -1.57, 1.57, -1.5, -1.57, 0.0]

    for joint_id, val in zip(arm_joints[:6], start_positions):
        p.resetJointState(robot_id, joint_id, val)

    # Enable rendering once setup is done
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Let physics settle
    for _ in range(120):
        p.stepSimulation()

    # Set GUI viewpoint
    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0.0, 0.0, 0.7]
    )

    return robot_id, object_ids


def take_snapshot(snapshot_path="pybullet_scene_snapshot.png", width=900, height=600):
    """
    Take a top-down snapshot of the current scene and save it.
    Call this AFTER setup_scene().
    """
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[0.0, 0.0, 1.8],
        cameraTargetPosition=[0.0, 0.0, 0.6],
        cameraUpVector=[0.0, 1.0, 0.0]
    )

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60.0,
        aspect=width / height,
        nearVal=0.1,
        farVal=5.0
    )

    img = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER
    )

    rgba = np.array(img[2], dtype=np.uint8).reshape(height, width, 4)
    rgb = rgba[:, :, :3]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    cv2.imwrite(snapshot_path, bgr)
    print(f"Snapshot saved: {snapshot_path}")


def get_arm_joints(robot_id):
    """Return list of revolute joint indices for the robot arm."""
    joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] == p.JOINT_REVOLUTE:
            joints.append(i)
    return joints


def get_world_bounds(canvas_w=900, canvas_h=600):
    """
    Estimate visible world bounds for the top-down camera.
    Useful for debugging pixel/world coordinate mapping.
    """
    cam_pos = [0.0, 0.0, 1.8]
    target_pos = [0.0, 0.0, 0.6]
    fov = 60.0

    cam_height = cam_pos[2] - 0.65
    half_h = cam_height * np.tan(np.radians(fov / 2))
    half_w = half_h * (canvas_w / canvas_h)

    print(f"[Calibration] Camera sees X: [{-half_w:.3f}, {half_w:.3f}]")
    print(f"[Calibration] Camera sees Y: [{-half_h:.3f}, {half_h:.3f}]")
    print("[Calibration] Paste these into task_planner.py:")
    print(f"  WORLD_X_MIN, WORLD_X_MAX = {-half_w:.3f}, {half_w:.3f}")
    print(f"  WORLD_Y_MIN, WORLD_Y_MAX = {half_h:.3f}, {-half_h:.3f}  # Y is flipped")

    return half_w, half_h