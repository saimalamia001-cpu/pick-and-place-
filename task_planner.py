import time
import math
import pybullet as p
from collections import namedtuple

CANVAS_W = 900
CANVAS_H = 600

WORLD_X_MIN, WORLD_X_MAX = -0.9730, 0.9730
WORLD_Y_MIN, WORLD_Y_MAX = 0.6640, -0.6640  # Y is flipped

TABLE_Z = 0.65
HOVER_Z = 1.00
GRASP_Z = 0.7
DROPOFF_Z = 0.75


def pixel_to_world(px, py, z=HOVER_Z):
    wx = WORLD_X_MIN + (px / CANVAS_W) * (WORLD_X_MAX - WORLD_X_MIN)
    wy = WORLD_Y_MIN + (py / CANVAS_H) * (WORLD_Y_MAX - WORLD_Y_MIN)
    return (wx, wy, z)


def world_to_pixel(wx, wy):
    px = int((wx - WORLD_X_MIN) / (WORLD_X_MAX - WORLD_X_MIN) * CANVAS_W)
    py = int((wy - WORLD_Y_MIN) / (WORLD_Y_MAX - WORLD_Y_MIN) * CANVAS_H)
    return (px, py)


class TaskPlanner:
    def __init__(self, robot_id, object_ids):
        self.robot_id = robot_id
        self.object_ids = object_ids
        self.selected_object_id = None

        self.eef_id = 7
        self.arm_num_dofs = 6
        self.arm_rest_poses = [0, -1.57, 1.57, -1.5, -1.57, 0.0]
        self.gripper_range = [0.0, 0.085]
        self.max_velocity = 3

        self._parse_joint_info()
        self._setup_mimic_joints()

        print(f"[TaskPlanner] arm joints:     {self.arm_controllable_joints}")
        print(f"[TaskPlanner] EE link index:  {self.eef_id}")

    def _parse_joint_info(self):
        JointInfo = namedtuple(
            "JointInfo",
            ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity", "controllable"]
        )

        self.joints = []
        self.controllable_joints = []

        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            controllable = info[2] != p.JOINT_FIXED

            j = JointInfo(
                id=info[0],
                name=info[1].decode("utf-8"),
                type=info[2],
                lowerLimit=info[8],
                upperLimit=info[9],
                maxForce=info[10],
                maxVelocity=info[11],
                controllable=controllable
            )
            self.joints.append(j)

            if controllable:
                self.controllable_joints.append(j.id)

        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]

        self.arm_lower_limits = [
            j.lowerLimit for j in self.joints if j.controllable
        ][:self.arm_num_dofs]

        self.arm_upper_limits = [
            j.upperLimit for j in self.joints if j.controllable
        ][:self.arm_num_dofs]

        self.arm_joint_ranges = [
            ul - ll for ul, ll in zip(self.arm_upper_limits, self.arm_lower_limits)
        ]

    def _setup_mimic_joints(self):
        mimic_parent_name = "finger_joint"
        mimic_children_names = {
            "right_outer_knuckle_joint": 1,
            "left_inner_knuckle_joint": 1,
            "right_inner_knuckle_joint": 1,
            "left_inner_finger_joint": -1,
            "right_inner_finger_joint": -1,
        }

        self.mimic_parent_id = [
            j.id for j in self.joints if j.name == mimic_parent_name
        ][0]

        self.mimic_child_multiplier = {
            j.id: mimic_children_names[j.name]
            for j in self.joints if j.name in mimic_children_names
        }

        for joint_id, multiplier in self.mimic_child_multiplier.items():
            c = p.createConstraint(
                self.robot_id,
                self.mimic_parent_id,
                self.robot_id,
                joint_id,
                jointType=p.JOINT_GEAR,
                jointAxis=[0, 1, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0]
            )
            p.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)

    def select_object_from_circle(self, circle_center_px, circle_radius_px):
        if circle_center_px is None or circle_radius_px is None:
            return None

        cx, cy = circle_center_px
        candidates = []

        for obj_id in self.object_ids:
            obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
            obj_px, obj_py = world_to_pixel(obj_pos[0], obj_pos[1])

            dist = ((obj_px - cx) ** 2 + (obj_py - cy) ** 2) ** 0.5

            if dist <= circle_radius_px:
                candidates.append((obj_id, dist, (obj_px, obj_py), obj_pos))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[1])
        best_obj_id, best_dist, best_px, best_pos = candidates[0]

        print(f"[TaskPlanner] Selected object ID: {best_obj_id}")
        print(f"[TaskPlanner] Selected object pixel: {best_px}")
        print(f"[TaskPlanner] Selected object world: {best_pos}")
        print(f"[TaskPlanner] Distance from circle center: {best_dist:.2f} px")

        return best_obj_id

    def execute(self, trajectory_data):
        grasp_circle_center = trajectory_data.get("grasp")
        grasp_circle_radius = trajectory_data.get("grasp_radius")
        dropoff_px = trajectory_data.get("dropoff")
        waypoints = trajectory_data.get("waypoints", [])

        if grasp_circle_center is None or grasp_circle_radius is None or dropoff_px is None:
            print("[TaskPlanner] Missing grasp circle, radius, or dropoff — aborting.")
            return

        self.selected_object_id = self.select_object_from_circle(
            grasp_circle_center, grasp_circle_radius
        )
        if self.selected_object_id is None:
            print("[TaskPlanner] No object found inside drawn circle.")
            return

        obj_pos, _ = p.getBasePositionAndOrientation(self.selected_object_id)

        grasp_hover = (obj_pos[0], obj_pos[1], HOVER_Z)
        grasp_down = (obj_pos[0], obj_pos[1], GRASP_Z)

        dropoff_hover = pixel_to_world(*dropoff_px, z=HOVER_Z)
        dropoff_down = pixel_to_world(*dropoff_px, z=DROPOFF_Z)
        wp_world = [pixel_to_world(*wp, z=HOVER_Z) for wp in waypoints]

        print(f"[TaskPlanner] Grasp hover:   {grasp_hover}")
        print(f"[TaskPlanner] Grasp down:    {grasp_down}")
        print(f"[TaskPlanner] Dropoff world: {dropoff_hover}")
        print(f"[TaskPlanner] Waypoints:     {len(wp_world)}")

        eef_state = p.getLinkState(self.robot_id, self.eef_id)
        eef_orn = eef_state[1]

        print("[TaskPlanner] Opening gripper...")
        self.move_gripper(0.085)
        self._step(240)

        print("[TaskPlanner] Moving above grasp...")
        self.move_arm_ik(grasp_hover, eef_orn)
        self._step(250)

        self.move_gripper(0.085)
        self._step(100)

        print("[TaskPlanner] Descending to grasp...")
        self.move_arm_ik(grasp_down, eef_orn)
        self._step(300)

        left_pad_before = p.getLinkState(self.robot_id, 12)[4]
        right_pad_before = p.getLinkState(self.robot_id, 17)[4]

        print(f"[TaskPlanner] Left pad position BEFORE close:  {left_pad_before}")
        print(f"[TaskPlanner] Right pad position BEFORE close: {right_pad_before}")

        print("[TaskPlanner] Closing gripper...")
        self.move_gripper(0.0)
        self._step(300)

        contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.selected_object_id)
        print(f"[TaskPlanner] Robot-object contacts: {len(contacts)}")
        for c in contacts:
            print(f"  robot link {c[3]} touching object, normal force = {c[9]:.4f}")

        left_pad_after = p.getLinkState(self.robot_id, 12)[4]
        right_pad_after = p.getLinkState(self.robot_id, 17)[4]

        gap = abs(left_pad_after[0] - right_pad_after[0])
        print(f"[TaskPlanner] Pad center gap after close: {gap:.4f} m")
        print(f"[TaskPlanner] Left pad position AFTER close:  {left_pad_after}")
        print(f"[TaskPlanner] Right pad position AFTER close: {right_pad_after}")

        left_contacts = p.getContactPoints(
            bodyA=self.robot_id,
            bodyB=self.selected_object_id,
            linkIndexA=12
        )
        right_contacts = p.getContactPoints(
            bodyA=self.robot_id,
            bodyB=self.selected_object_id,
            linkIndexA=17
        )

        print(f"[TaskPlanner] Left pad contacts:  {len(left_contacts)}")
        print(f"[TaskPlanner] Right pad contacts: {len(right_contacts)}")

        print("[TaskPlanner] Lifting...")
        self.move_arm_ik(grasp_hover, eef_orn)
        self._step(200)

        print(f"[TaskPlanner] Following {len(wp_world)} waypoints...")
        for i, wp in enumerate(wp_world):
            print(f"  waypoint {i + 1}/{len(wp_world)}: {wp}")
            self.move_arm_ik(wp, eef_orn)
            self._step(150)

        print("[TaskPlanner] Moving above dropoff...")
        self.move_arm_ik(dropoff_hover, eef_orn)
        self._step(200)

        print("[TaskPlanner] Descending to dropoff...")
        self.move_arm_ik(dropoff_down, eef_orn)
        self._step(150)

        print("[TaskPlanner] Releasing object...")
        self.move_gripper(0.085)
        self._step(100)

        print("[TaskPlanner] Retreating...")
        self.move_arm_ik(dropoff_hover, eef_orn)
        self._step(150)

        print("[TaskPlanner] Task complete.")

    def move_arm_ik(self, target_pos, target_orn, threshold=0.02, max_steps=500):
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.eef_id,
            target_pos,
            target_orn,
            lowerLimits=self.arm_lower_limits,
            upperLimits=self.arm_upper_limits,
            jointRanges=self.arm_joint_ranges,
            restPoses=self.arm_rest_poses,
            maxNumIterations=200,
            residualThreshold=1e-4
        )

        for i, joint_id in enumerate(self.arm_controllable_joints):
            p.setJointMotorControl2(
                self.robot_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=400,
                maxVelocity=1.0
            )

        for step in range(max_steps):
            p.stepSimulation()
            time.sleep(1.0 / 120.0)

            ee_state = p.getLinkState(self.robot_id, self.eef_id)
            ee_pos = ee_state[4]
            dist = (
                (ee_pos[0] - target_pos[0]) ** 2 +
                (ee_pos[1] - target_pos[1]) ** 2 +
                (ee_pos[2] - target_pos[2]) ** 2
            ) ** 0.5

            if dist < threshold:
                print(f"  reached target in {step + 1} steps, dist={dist:.4f}")
                break
        else:
            ee_state = p.getLinkState(self.robot_id, self.eef_id)
            print(f"  max steps hit, final dist={dist:.4f}, EE Z={ee_state[4][2]:.4f}")

    def move_gripper(self, open_length):
        open_length = max(self.gripper_range[0], min(open_length, self.gripper_range[1]))
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)

        p.setJointMotorControl2(
            self.robot_id,
            self.mimic_parent_id,
            p.POSITION_CONTROL,
            targetPosition=open_angle,
            force=300,
            maxVelocity=1.0
        )

    def _step(self, steps, sleep=1.0 / 120.0):
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(sleep)