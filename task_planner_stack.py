import time
import math
import pybullet as p
from collections import namedtuple

CANVAS_W = 900
CANVAS_H = 600

WORLD_X_MIN, WORLD_X_MAX = -0.9730, 0.9730
WORLD_Y_MIN, WORLD_Y_MAX = 0.6640, -0.6640  # Y is flipped

TABLE_Z    = 0.65
HOVER_Z    = 1.00
GRASP_Z    = 0.70



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
        self.robot_id   = robot_id
        self.object_ids = object_ids

        self.eef_id          = 7
        self.arm_num_dofs    = 6
        self.arm_rest_poses  = [0, -1.57, 1.57, -1.5, -1.57, 0.0]
        self.gripper_range   = [0.0, 0.085]

        self._parse_joint_info()
        self._setup_mimic_joints()

        print(f"[TaskPlanner] arm joints:    {self.arm_controllable_joints}")
        print(f"[TaskPlanner] EE link index: {self.eef_id}")

    # ── Joint setup (unchanged) ───────────────────────────────────────────────
    def _parse_joint_info(self):
        JointInfo = namedtuple(
            "JointInfo",
            ["id", "name", "type", "lowerLimit", "upperLimit",
             "maxForce", "maxVelocity", "controllable"]
        )
        self.joints             = []
        self.controllable_joints = []

        for i in range(p.getNumJoints(self.robot_id)):
            info         = p.getJointInfo(self.robot_id, i)
            controllable = info[2] != p.JOINT_FIXED
            j = JointInfo(
                id=info[0], name=info[1].decode("utf-8"), type=info[2],
                lowerLimit=info[8], upperLimit=info[9],
                maxForce=info[10], maxVelocity=info[11],
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
        mimic_parent_name    = "finger_joint"
        mimic_children_names = {
            "right_outer_knuckle_joint":  1,
            "left_inner_knuckle_joint":   1,
            "right_inner_knuckle_joint":  1,
            "left_inner_finger_joint":   -1,
            "right_inner_finger_joint":  -1,
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
                self.robot_id, self.mimic_parent_id,
                self.robot_id, joint_id,
                jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0]
            )
            p.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)

    # ── Object selection ──────────────────────────────────────────────────────
    def select_object_near_point(self, point_px, search_radius_px=60):
        """
        Find the object whose world position projects closest to point_px.
        Used for both arrow-tail (source) and arrow-head / triangle-tip (destination).
        search_radius_px: how many pixels around the point to accept candidates.
        """
        if point_px is None:
            return None

        cx, cy = point_px
        candidates = []

        for obj_id in self.object_ids:
            obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
            obj_px, obj_py = world_to_pixel(obj_pos[0], obj_pos[1])
            dist = math.hypot(obj_px - cx, obj_py - cy)
            if dist <= search_radius_px:
                candidates.append((obj_id, dist, (obj_px, obj_py), obj_pos))

        if not candidates:
            print(f"[TaskPlanner] No object found within {search_radius_px}px of {point_px}")
            return None

        candidates.sort(key=lambda x: x[1])
        best_id, best_dist, best_px, best_pos = candidates[0]

        print(f"[TaskPlanner] Found object ID: {best_id}")
        print(f"[TaskPlanner]   pixel pos:  {best_px}  (query was {point_px})")
        print(f"[TaskPlanner]   world pos:  {best_pos}")
        print(f"[TaskPlanner]   distance:   {best_dist:.1f} px")
        return best_id

    def get_top_coordinate(self, obj_id):
        """
        Return the world (x, y, top_z) of an object using its AABB.
        This is where the source object will be placed.
        """
        aabb      = p.getAABB(obj_id)
        obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
        top_z     = aabb[1][2]          # max z of bounding box
        print(f"[TaskPlanner] Destination AABB top z: {top_z:.4f}")
        return (obj_pos[0], obj_pos[1], top_z)

    # ── Main stacking execute ─────────────────────────────────────────────────
    def execute(self, trajectory_data):
        """
        GUI records:
            waypoints   → freehand sampled points  (first point = start of curved arrow)
            dropoff     → triangle tip first click  (nearest object = destination)

        Source object  → object nearest to waypoints[0]  (first drawn freehand point)
        Destination    → object nearest to triangle_tip   (same logic as pick & place dropoff)
        """
        waypoints_px    = trajectory_data.get("waypoints", [])
        triangle_tip_px = trajectory_data.get("dropoff")    # first triangle click

        # Source: first freehand point drawn — start of the curved arrow
        source_px = waypoints_px[0] if waypoints_px else None

        if source_px is None:
            print("[TaskPlanner] No freehand waypoints drawn — cannot find source object.")
            return
        if triangle_tip_px is None:
            print("[TaskPlanner] No triangle tip drawn — cannot find destination object.")
            return

        # ── 1. Find source object (near arrow tail) ───────────────────────────
        print("\n[TaskPlanner] === STACKING TASK ===")
        print(f"[TaskPlanner] Source px (waypoints[0]): {source_px}")
        print(f"[TaskPlanner] Triangle tip px:          {triangle_tip_px}")

        # ── 1. Find source object (nearest to first waypoint) ─────────────────
        source_id = self.select_object_near_point(source_px, search_radius_px=80)
        if source_id is None:
            print("[TaskPlanner] Could not find source object near first waypoint.")
            return

        # ── 2. Find destination object (nearest to triangle tip) ──────────────
        dest_id = self.select_object_near_point(triangle_tip_px, search_radius_px=80)
        if dest_id is None:
            print("[TaskPlanner] Could not find destination object near triangle tip.")
            return

        if source_id == dest_id:
            print("[TaskPlanner] Source and destination are the same object — aborting.")
            return

        print(f"[TaskPlanner] Source obj ID:      {source_id}")
        print(f"[TaskPlanner] Destination obj ID: {dest_id}")

        # ── 3. Get poses ──────────────────────────────────────────────────────
        src_pos, _ = p.getBasePositionAndOrientation(source_id)
        dest_top   = self.get_top_coordinate(dest_id)   # (x, y, top_z)

        # Source: hover above, then descend to grasp
        src_hover  = (src_pos[0],   src_pos[1],   HOVER_Z)
        src_down   = (src_pos[0],   src_pos[1],   GRASP_Z)

        # Destination: hover with extra clearance (carrying an object), then place on top
        dest_place_z    = dest_top[2] + 0.07    # just above destination top surface
        dest_hover = (dest_top[0], dest_top[1], HOVER_Z)
        dest_place      = (dest_top[0], dest_top[1], dest_place_z)

        # Waypoints along the curved arrow path (at hover height)
        wp_world = [pixel_to_world(*wp, z=HOVER_Z) for wp in waypoints_px]

        print(f"[TaskPlanner] Source hover:   {src_hover}")
        print(f"[TaskPlanner] Source grasp:   {src_down}")
        print(f"[TaskPlanner] Dest hover:     {dest_hover}")
        print(f"[TaskPlanner] Dest place:     {dest_place}")
        print(f"[TaskPlanner] Waypoints:      {len(wp_world)}")

        eef_orn = p.getLinkState(self.robot_id, self.eef_id)[1]


        # ── 4. Open gripper ───────────────────────────────────────────────────
        print("\n[TaskPlanner] Opening gripper...")
        self.move_gripper(0.085)
        self._step(240)

        # ── 5. Move above source ──────────────────────────────────────────────
        print("[TaskPlanner] Moving above source object...")
        self.move_arm_ik(src_hover, eef_orn)
        self._step(250)

        # ── 6. Descend and grasp ──────────────────────────────────────────────
        print("[TaskPlanner] Descending to grasp...")
        self.move_arm_ik(src_down, eef_orn)
        self._step(300)

        print("[TaskPlanner] Closing gripper...")
        self.move_gripper(0.0)
        self._step(300)

        contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=source_id)
        print(f"[TaskPlanner] Robot-object contacts after grasp: {len(contacts)}")

        # ── 7. Lift source object ─────────────────────────────────────────────
        print("[TaskPlanner] Lifting source object...")
        self.move_arm_ik(src_hover, eef_orn)
        self._step(250)

        # ── 8. Follow waypoints toward destination ────────────────────────────
        print(f"[TaskPlanner] Following {len(wp_world)} waypoints...")
        for i, wp in enumerate(wp_world):
            print(f"  waypoint {i+1}/{len(wp_world)}: {wp}")
            self.move_arm_ik(wp, eef_orn)
            self._step(150)

        # ── 9. Hover above destination ────────────────────────────────────────
        print("[TaskPlanner] Moving above destination...")
        self.move_arm_ik(dest_hover, eef_orn)
        self._step(200)

        # ── 10. Lower onto destination top surface ────────────────────────────
        print(f"[TaskPlanner] Lowering onto destination (place z={dest_place[2]:.4f})...")
        self.move_arm_ik(dest_place, eef_orn)
        self._step(200)

        # ── 11. Release ───────────────────────────────────────────────────────
        print("[TaskPlanner] Releasing object...")
        self.move_gripper(0.085)
        self._step(150)

        # ── 12. Retreat ───────────────────────────────────────────────────────
        print("[TaskPlanner] Retreating...")
        self.move_arm_ik(dest_hover, eef_orn)
        self._step(200)

        # ── 13. Verify stack ──────────────────────────────────────────────────
        src_final, _ = p.getBasePositionAndOrientation(source_id)
        dst_final, _ = p.getBasePositionAndOrientation(dest_id)
        height_diff  = src_final[2] - dst_final[2]
        print(f"\n[TaskPlanner] === RESULT ===")
        print(f"[TaskPlanner] Source final Z:      {src_final[2]:.4f}")
        print(f"[TaskPlanner] Destination final Z: {dst_final[2]:.4f}")
        print(f"[TaskPlanner] Height difference:   {height_diff:.4f}  "
              f"({'stacked ✓' if height_diff > 0.03 else 'may not be stacked ✗'})")

    # ── IK / gripper / step (unchanged from original) ─────────────────────────
    def move_arm_ik(self, target_pos, target_orn, threshold=0.02, max_steps=500):
        joint_poses = p.calculateInverseKinematics(
            self.robot_id, self.eef_id,
            target_pos, target_orn,
            lowerLimits=self.arm_lower_limits,
            upperLimits=self.arm_upper_limits,
            jointRanges=self.arm_joint_ranges,
            restPoses=self.arm_rest_poses,
            maxNumIterations=200,
            residualThreshold=1e-4
        )
        for i, joint_id in enumerate(self.arm_controllable_joints):
            p.setJointMotorControl2(
                self.robot_id, joint_id,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=400, maxVelocity=1.0
            )
        dist = 999
        for step in range(max_steps):
            p.stepSimulation()
            time.sleep(1.0 / 120.0)
            ee_pos = p.getLinkState(self.robot_id, self.eef_id)[4]
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(ee_pos, target_pos)))
            if dist < threshold:
                print(f"  reached in {step+1} steps, dist={dist:.4f}")
                break
        else:
            print(f"  max steps hit, dist={dist:.4f}")

    def move_gripper(self, open_length):
        open_length = max(self.gripper_range[0], min(open_length, self.gripper_range[1]))
        open_angle  = 0.715 - math.asin((open_length - 0.010) / 0.1143)
        p.setJointMotorControl2(
            self.robot_id, self.mimic_parent_id,
            p.POSITION_CONTROL,
            targetPosition=open_angle,
            force=300, maxVelocity=1.0
        )

    def _step(self, steps, sleep=1.0 / 120.0):
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(sleep)
