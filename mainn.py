import pybullet as p
from simulation import setup_scene, take_snapshot
from gui_drawing import launch_gui
from task_planner import TaskPlanner


def main():
    snapshot_path = "pybullet_scene_snapshot.png"

    print("Setting up simulation...")
    robot_id, object_ids = setup_scene()

    # Check object dimensions
    for i, obj_id in enumerate(object_ids, start=1):
        aabb = p.getAABB(obj_id)
        center_z = (aabb[0][2] + aabb[1][2]) / 2
        top_z = aabb[1][2]
        print(f"Object {i} (id={obj_id}) AABB min: {aabb[0]}")
        print(f"Object {i} (id={obj_id}) AABB max: {aabb[1]}")
        print(f"Object {i} center Z: {center_z:.4f}")
        print(f"Object {i} top Z:    {top_z:.4f}")

    take_snapshot(snapshot_path)

    print("Opening drawing GUI... draw your trajectory then click Done.")
    trajectory_data = launch_gui(background_path=snapshot_path)

    print("\nTrajectory data received:")
    print(f"  Grasp:        {trajectory_data.get('grasp')}")
    print(f"  Grasp radius: {trajectory_data.get('grasp_radius')}")
    print(f"  Dropoff:      {trajectory_data.get('dropoff')}")
    print(f"  Waypoints:    {len(trajectory_data.get('waypoints', []))} points")

    if trajectory_data.get("grasp") and trajectory_data.get("dropoff"):
        planner = TaskPlanner(robot_id, object_ids)
        planner.execute(trajectory_data)
    else:
        print("No grasp or dropoff drawn — nothing to execute.")

    input("Press Enter to close simulation...")
    p.disconnect()


if __name__ == "__main__":
    main()