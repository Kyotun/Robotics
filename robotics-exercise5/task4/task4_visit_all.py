#!/usr/bin/env python3
"""
Task 4 — Visit-All with Unified Planning (UPF) + PyRoboSim

This script:
  - builds/loads a PyRoboSim world,
  - loads PDDL domain/problem from files (REQUIRED),
  - solves with a UPF oneshot planner (e.g., pyperplan),
  - executes the plan by navigating the robot room-to-room.

Usage:
  python task4_visit_all.py --domain-file visit_all_domain.pddl --problem-file visit_all_problem.pddl
  python task4_visit_all.py --world-file example_world.yaml --domain-file ... --problem-file ...
  python task4_visit_all.py --no-gui --domain-file ... --problem-file ...
  python task4_visit_all.py --engine pyperplan --domain-file ... --problem-file ...
"""

import os
import argparse
import time
from typing import List, Tuple

# PyRoboSim 
from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.a_star import AStarPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Unified Planning
import unified_planning as up
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner


DATA_FOLDER = get_data_folder()


# ----------------- CLI -----------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Task 4 — Visit-All (UPF + PyRoboSim) with external PDDL files")
    p.add_argument("--world-file", default="", help="YAML file (under pyrobosim/data) to load a world.")
    p.add_argument("--no-gui", action="store_true", help="Run headless (no GUI).")
    p.add_argument("--engine", default="pyperplan", help="UPF oneshot planner name (e.g., pyperplan, fast-downward, aries).")
    p.add_argument("--robot", default="", help="Robot name to use (default: first robot in world).")
    p.add_argument("--partial-obs-objects", action="store_true",
                   help="If True, robots have partial observability of objects and must detect them.",
    )

    # REQUIRED: PDDL file inputs
    p.add_argument("--domain-file", required=True, help="Path to visit-all domain PDDL.")
    p.add_argument("--problem-file", required=True, help="Path to visit-all problem PDDL.")

    return p.parse_args()


# ----------------- World helpers -----------------
def create_world_from_yaml(world_file: str) -> World:
    return WorldYamlLoader().from_file(os.path.join(DATA_FOLDER, world_file))


def create_world(multirobot: bool = False) -> World:
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.add_metadata(
        locations=[
            os.path.join(DATA_FOLDER, "example_location_data_furniture.yaml"),
            os.path.join(DATA_FOLDER, "example_location_data_accessories.yaml"),
        ],
        objects=[
            os.path.join(DATA_FOLDER, "example_object_data_food.yaml"),
            os.path.join(DATA_FOLDER, "example_object_data_drink.yaml"),
        ],
    )

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    world.add_room(
        name="kitchen",
        pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r1coords,
        color="red",
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
    r2coords = [(-0.875, -0.75), (0.875, -0.75), (0.875, 0.75), (-0.875, 0.75)]
    world.add_room(
        name="office1",
        pose=Pose(x=2.625, y=3.25, z=0.0, yaw=0.0),
        footprint=r2coords,
        color="#009900",
    )
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(
        name="bathroom",
        footprint=r3coords,
        color=[0.0, 0.0, 0.6],
    )
    office2_coords = [(-1.0, -0.75), (1.0, -0.75), (1.0, 0.75), (-1.0, 0.75)]
    world.add_room(
        name="office2",
        pose=Pose(x=4.0, y=1.0),
        footprint=office2_coords,
        color="#3366FF"
    )
    

    # Add hallways between the rooms
    world.add_hallway(
        room_start="kitchen", room_end="bathroom", width=0.7, color="#666666"
    )
    world.add_hallway(
        room_start="bathroom",
        room_end="office1",
        width=0.5,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
        color="dimgray",
    )
    world.add_hallway(
        room_start="kitchen",
        room_end="office1",
        width=0.6,
        conn_method="points",
        conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
    )
    world.add_hallway(
        room_start="office1",
        room_end="office2",
        width=0.6,
        conn_method="points",
        conn_points=[(3.5, 3.25), (4.0, 2.0), (4.0, 1.0)],
        color="#444444"
    )

   

    # Add locations
    table = world.add_location(
        category="table",
        parent="kitchen",
        pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-90.0, angle_units="degrees"),
    )
    desk_pose = world.get_pose_relative_to(
        Pose(x=0.525, y=0.4, z=0.0, yaw=0.0), "office1"
    )
    desk = world.add_location(category="desk", parent="office1", pose=desk_pose)

   

    counter = world.add_location(
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    # Add objects
    banana_pose = world.get_pose_relative_to(
        Pose(x=0.15, y=0.0, z=0.0, q=[0.9238811, 0.0, 0.0, -0.3826797]), table
    )
    world.add_object(category="banana", parent=table, pose=banana_pose)
    apple_pose = world.get_pose_relative_to(
        Pose(x=0.05, y=-0.15, z=0.0, q=[1.0, 0.0, 0.0, 0.0]), desk
    )
    world.add_object(category="apple", parent=desk, pose=apple_pose)
    world.add_object(category="apple", parent=table)
    world.add_object(category="apple", parent=table)
    world.add_object(category="water", parent=counter)
    world.add_object(category="banana", parent=counter)
    world.add_object(category="water", parent=desk)

    # Add robots
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    lidar = Lidar2D(
        update_rate_s=0.1,
        angle_units="degrees",
        min_angle=-120.0,
        max_angle=120.0,
        angular_resolution=5.0,
        max_range_m=2.0,
    )

    robot0 = Robot(
        name="robot0",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            validate_during_execution=True,
        ),
        sensors={"lidar": lidar},
        grasp_generator=GraspGenerator(grasp_props),
        partial_obs_objects=args.partial_obs_objects,
        color="#CC00CC",
    )
    world.add_robot(robot0, loc="kitchen")
    planner_config_rrt = {
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    rrt_planner = RRTPlanner(**planner_config_rrt)
    robot0.set_path_planner(rrt_planner)

    if multirobot:
        robot1 = Robot(
            name="robot1",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_obs_objects=args.partial_obs_objects,
        )
        world.add_robot(robot1, loc="bathroom")
        planner_config_prm = {
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 1.5,
            "max_nodes": 100,
            "compress_path": False,
        }
        prm_planner = PRMPlanner(**planner_config_prm)
        robot1.set_path_planner(prm_planner)

        robot2 = Robot(
            name="robot2",
            radius=0.06,
            color=(0, 0.8, 0.8),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_obs_objects=args.partial_obs_objects,
        )
        world.add_robot(robot2, loc="office1")
        planner_config_astar = {
            "grid_resolution": 0.05,
            "grid_inflation_radius": 0.15,
            "diagonal_motion": True,
            "heuristic": "euclidean",
        }
        astar_planner = AStarPlanner(**planner_config_astar)
        robot2.set_path_planner(astar_planner)

    return world


def get_robot(world: World, preferred_name: str = "") -> Robot:
    if preferred_name:
        for robot in world.robots:
            if robot.name == preferred_name:
                return robot
        raise ValueError(f"Robot '{preferred_name}' not found. Available: {[robot.name for robot in world.robots]}")
    if not world.robots:
        raise RuntimeError("No robots in world. Add one with world.add_robot(...)")
    return world.robots[0]


def room_nav_pose(world: World, room_name: str) -> Pose:
    """Pick a reasonable navigation target pose for a room."""
    room = world.get_room(room_name)
    if room is None:
        return Pose(0.0, 0.0)
    # Prefer an explicitly defined nav pose
    try:
        if room.nav_poses:
            return room.nav_poses[0]
    except Exception:
        pass
    # Fallback: centroid of footprint
    try:
        xs = [fprnt[0] for fprnt in room.footprint]
        ys = [fprnt[1] for fprnt in room.footprint]
        return Pose(sum(xs) / len(xs), sum(ys) / len(ys))
    except Exception:
        return Pose(0.0, 0.0)


# ----------------- Execution -----------------
def navigate_to_room(world: World, robot: Robot, room_name: str, block: bool = True, dt: float = 0.05, timeout_s: float = 60.0):
    """Command the robot to navigate to a room's nav pose. Poll the world until idle."""
    goal_pose = room_nav_pose(world, room_name)
    robot.navigate_to(goal_pose)

    if not block:
        return

    t0 = time.time()
    while True:
        # Check if executor exposes an "is_executing" flag
        busy = True
        try:
            if hasattr(robot, "path_executor") and hasattr(robot.path_executor, "is_executing"):
                busy = robot.path_executor.is_executing
        except Exception:
            busy = True

        if not busy:
            break

        # Step simulation or sleep if GUI thread owns stepping
        try:
            world.update(dt)
        except Exception:
            time.sleep(dt)

        if time.time() - t0 > timeout_s:
            print(f"[WARN] Timeout navigating to {room_name}")
            break


def execute_visit_all(world: World, plan_steps):
    """Map PDDL 'move(my_robot, from, to)' to PyRoboSim navigation."""
    robot = get_robot(world)
    for name, params in plan_steps:
        if name != "move":
            print(f"[WARN] Skipping non-move action: {name}")
            continue
        _, frm, to = params
        print(f"[EXEC] move: {frm} -> {to}")
        navigate_to_room(world, robot, to, block=True)


if __name__ == "__main__":
    args = parse_args()

    # Silence UP credits banner (optional)
    up.shortcuts.get_environment().credits_stream = None

    # Build/load world
    world = create_world_from_yaml(args.world_file) if args.world_file else create_world()

    # Start GUI unless headless
    if not args.no_gui:
        start_gui(world)

    # Load PDDL from files and solve
    reader = PDDLReader()
    problem = reader.parse_problem(args.domain_file, args.problem_file)

    try:
        with OneshotPlanner(name=args.engine, problem_kind=problem.kind) as planner:
            print(f"[UPF] Using engine: {planner.name}")
            result = planner.solve(problem)
    except Exception as e:
        # Helpful fallback and hint
        print(f"[UPF] Engine '{args.engine}' not available: {e}")
        print("Try:  pip install up-pyperplan   (or up-fast-downward / up-aries)")
        # Also try auto-pick if possible
        with OneshotPlanner(problem_kind=problem.kind) as planner:
            print(f"[UPF] Auto-picked engine: {planner.name}")
            result = planner.solve(problem)

    if result.plan is None:
        raise RuntimeError("No plan found. Check your domain/problem files and connectivity.")

    # Extract steps
    plan_steps = []
    print("[UPF] Plan:")
    for ai in result.plan.actions:
        name = ai.action.name
        params = [str(o) for o in ai.actual_parameters]
        plan_steps.append((name, params))
        print("  ", name, tuple(params))

    # Execute
    execute_visit_all(world, plan_steps)
    print("[DONE] Task 4 execution finished.")
