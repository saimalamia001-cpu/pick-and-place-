import pybullet as p
from simulation_debug import setup_scene, take_snapshot
from gui_drawing import launch_gui
from task_planner_stack import TaskPlanner


def main():
    snapshot_path = "pybullet_scene_snapshot.png"

    print("Setting up simulation...")
    robot_id, object_ids = setup_scene()

    print("\n[Scene] Object dimensions:")
    for i, obj_id in enumerate(object_ids, start=1):
        aabb   = p.getAABB(obj_id)
        top_z  = aabb[1][2]
        pos, _ = p.getBasePositionAndOrientation(obj_id)
        print(f"  Object {i} (id={obj_id})  world pos={pos}  top_z={top_z:.4f}")

    take_snapshot(snapshot_path)

    print("\nOpening drawing GUI...")
    print("  1. Select FREEHAND tool — draw a curved line STARTING over the source object.")
    print("     The first point you draw = grasp object.")
    print("  2. Select TRIANGLE tool — first click goes above the destination object.")
    print("     The nearest object to that first click = stack destination.")
    print("  Click Done when finished.\n")

    trajectory_data = launch_gui(background_path=snapshot_path)

    waypoints    = trajectory_data.get("waypoints", [])
    triangle_tip = trajectory_data.get("dropoff")

    print("\n[Main] Trajectory data received:")
    print(f"  First waypoint (source search): {waypoints[0] if waypoints else None}")
    print(f"  Total waypoints:                {len(waypoints)} points")
    print(f"  Triangle tip (dest search):     {triangle_tip}")

    if waypoints and triangle_tip:
        planner = TaskPlanner(robot_id, object_ids)
        planner.execute(trajectory_data)
    else:
        if not waypoints:
            print("[Main] No freehand line drawn — cannot find source object.")
        if not triangle_tip:
            print("[Main] No triangle tip drawn — cannot find destination object.")

    input("\nPress Enter to close simulation...")
    p.disconnect()


if __name__ == "__main__":
    main()
