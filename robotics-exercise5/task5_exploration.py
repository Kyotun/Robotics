#!/usr/bin/env python3
"""
Task 5 — Exploration and Dynamic Planning with PyRoboSim + Unified Planning

Steps:
  1. Load world (with --world-file).
  2. Perform visit-all exploration (move through rooms).
  3. Record discovered objects.
  4. Generate a new PDDL problem: deliver all objects to a drop location.
  5. Solve dynamically with Unified Planning (UPF).
  6. Execute resulting plan (navigate, pick, place).
"""

import os
import time
import argparse
import threading

# PyRoboSim 
from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
from pyrobosim.core.room import Room
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Unified Planning
import unified_planning as up
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner

DATA_FOLDER = get_data_folder()


# ----------------- CLI -----------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Task 5 — Exploration + Dynamic Replanning")
    p.add_argument("--world-file", required=True, help="World YAML file")
    p.add_argument("--domain-file", default="exploration_domain.pddl", help="Path to domain PDDL (pick/place domain)")
    p.add_argument("--drop-location", default="trash", help="Name of drop location (surface) for all items")
    p.add_argument("--no-gui", action="store_true", help="Run without GUI")
    p.add_argument("--engine", default="pyperplan", help="UPF oneshot planner")
    return p.parse_args()


# ----------------- World helpers -----------------
def create_world_from_yaml(fname: str) -> World:
    return WorldYamlLoader().from_file(f"{fname}.yaml")
    if os.path.isabs(fname) or os.path.exists(fname):
        return WorldYamlLoader().from_file(fname)
    return WorldYamlLoader().from_file(os.path.join(DATA_FOLDER, fname))


def getRoomByName(world: World, room_name:str) -> Room | None:
    for room in world.rooms:
        if getattr(room, "name", None) == room_name:
            return room
    return None

def get_robot(world: World) -> Robot:
    if not world.robots:
        raise RuntimeError("No robots in world. Add one with world.add_robot(...)")
    return world.robots[0]


def room_nav_pose(world: World, room_name: str) -> Pose:
    room = world.get_room_by_name(room_name)
    if room and room.nav_poses:
        return room.nav_poses[0]
    return Pose(0.0, 0.0)  # fallback


def navigate_to_room(world: World, robot: Robot, room_name: str, block=True):
    goal_pose = room_nav_pose(world, room_name)
    robot.navigate(goal=goal_pose)

    if not block:
        return
    while getattr(robot.path_executor, "is_executing", False):
        try:
            world.update(0.1)
        except Exception:
            time.sleep(0.1)


# ----------------- Task 5 core logic -----------------
def explore_and_find_objects(world: World, robot: Robot):
    """Visit all rooms & record objects discovered in them"""
    discovered = []
    for room in world.rooms:
        print(f"[EXPLORE] Going to {room.name}")
        navigate_to_room(world, robot, room.name)
        objs = [obj for obj in world.objects if obj.parent == room]
        for obj in objs:
            discovered.append((obj.name, obj.category, room, obj.parent))
            print(f"[FOUND] {obj.name} ({obj.category}) in {room}, parent {obj.parent}")
    return discovered


def generate_dynamic_problem(domain_file: str, objects, drop_location: str, init_room="kitchen"):
    """
    Generate a new PDDL problem string:
      - deliver all discovered objects to drop_location
    """
    rooms = set([room for (_,_,room,_) in objects] + [init_room])
    places = set([place for (*_,place) in objects] + [drop_location])
    items = set([obj for (obj,_,_,_) in objects])

    init_atoms = []
    init_atoms.append(f"(at my_robot {init_room})")
    init_atoms.append("(handempty my_robot)")
    for obj, cat, room, place in objects:
        init_atoms.append(f"(on {obj} {place})")
        init_atoms.append(f"(locationof {place} {room})")
    init_atoms.append(f"(locationof {drop_location} {init_room})")

    # Simple full connectivity (assume all rooms connected)
    for a in rooms:
        for b in rooms:
            if a != b:
                init_atoms.append(f"(connected {a} {b})")

    goal_atoms = [f"(on {obj} {drop_location})" for obj in items]

    problem_str = f"""
(define (problem task5-dynamic)
  (:domain exploration-pickplace)

  (:objects
    my_robot - robot
    {' '.join(sorted(rooms))} - room
    {' '.join(sorted(places))} - place
    {' '.join(sorted(items))} - item
  )

  (:init
    {" ".join(init_atoms)}
  )

  (:goal
    (and {" ".join(goal_atoms)})
  )
)
"""
    fname = "dynamic_task5_problem.pddl"
    with open(fname, "w") as f:
        f.write(problem_str)
    print(f"[GENERATED] Problem file: {fname}")
    return fname

def execute_plan(world: World, plan):
    robot = get_robot(world)  # from your helper
    
    for act in plan.actions:
        name = act.action.name
        params = [str(p) for p in act.actual_parameters]
        print(f"[EXEC] {name} {params}")

        if name == "move":
            # Plan: (move my_robot from_room to_room)
            _, frm, to = params
            goal_pose = room_nav_pose(world, to)   
            robot.navigate_to(goal_pose)

            while getattr(robot.path_executor, "is_executing", False):
                world.update(0.1)

        elif name == "pick":
            _, obj, place, room = params
            if obj in world.objects_by_name:
                print(f"  -> Picking {obj}")
                robot.pick_object(obj)
            else:
                print(f"[WARN] Object {obj} not found in world!")

        elif name == "put":
            _, obj, place, room = params
            if obj in world.objects_by_name and place in world.locations_by_name:
                print(f"  -> Placing {obj} on {place}")
                robot.place_object(obj, place)
            else:
                print(f"[WARN] Could not place {obj} on {place}")
if __name__ == "__main__":
    args = parse_args()

    # --- Load world
    world = create_world_from_yaml(args.world_file)

    robot = get_robot(world)

   
    from pyrobosim.navigation.rrt import RRTPlanner
    from pyrobosim.navigation.execution import ConstantVelocityExecutor

    planner_config = {
        "bidirectional": True,
        "rrt_star": True,
        "max_connection_dist": 0.5,
        "collision_check_step_dist": 0.025,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    robot.set_path_planner(RRTPlanner(**planner_config))

    robot.set_path_executor(ConstantVelocityExecutor(
        linear_velocity=1.0,
        max_angular_velocity=4.0,
        dt=0.1,
        validate_during_execution=True,
    ))
  
    def thread_func():
            # --- Step 1: Exploration
        discovered = explore_and_find_objects(world, robot)

        # --- Step 2: Dynamic problem generation
        problem_file = generate_dynamic_problem(args.domain_file, discovered, args.drop_location)

        # --- Step 3: Solve with UPF
        reader = PDDLReader()
        problem = reader.parse_problem(args.domain_file, problem_file)

        with OneshotPlanner(name=args.engine, problem_kind=problem.kind) as planner:
            print(f"[UPF] Using {planner.name}")
            result = planner.solve(problem)

        if result.plan is None:
            raise RuntimeError("[ERROR] Could not find a plan for dynamic task!")

        print("[PLAN] Dynamic plan:")
        print(result.plan)

        # --- Step 4: Execute
        execute_plan(world, result.plan)

    threading.Thread(target=thread_func, daemon=True).start()
    if not args.no_gui:
        start_gui(world)

    print("[DONE] Task 5 completed successfully.")
