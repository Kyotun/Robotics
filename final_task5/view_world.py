
import os
import argparse
import math

from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation import PathPlanner
from pyrobosim.navigation.a_star import AStarPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.planning.actions import TaskAction, TaskPlan
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner
import yaml
from shapely.geometry import Point


data_folder = get_data_folder()

def compare(s1, s2):
  return s1[:5] == s2[:5]


def create_world(multirobot: bool = False) -> World:
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.add_metadata(
        locations=[
            os.path.join(data_folder, "example_location_data_furniture.yaml"),
            os.path.join(data_folder, "example_location_data_accessories.yaml"),
        ],
        objects=[
            os.path.join(data_folder, "example_object_data_food.yaml"),
            os.path.join(data_folder, "example_object_data_drink.yaml"),
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
        sensors={"lidar": lidar} if args.lidar else None,
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
        "max_connection_dist": 10,
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

def solve_with_upf(domain_pddl, problem_pddl):
    print("PHASE 3.2: SOLVING PROBLEM WITH UPF...")
    reader = PDDLReader()
    problem = reader.parse_problem(domain_pddl, problem_pddl)

    with OneshotPlanner(problem_kind=problem.kind) as planner:
        result = planner.solve(problem)
        if result.plan:
            print("  UPF found a plan!")
            for action in result.plan.actions:
                print(f"  - {action}")
            return result.plan
        else:
            print("  UPF could not find a plan.")
            return None
        
def execute_upf_plan(world, plan):
    if not plan:
        print("No plan to execute.")
        return

    print("\nPHASE 4: EXECUTING THE FINAL PLAN...")
    robot = world.robots[0]

    # can be used to quickly look up locations and objects by name
    location_map = {loc.name: loc for loc in world.locations}
    object_map = {obj.name: obj for obj in world.objects}

    # iterate through each action in the UPF plan
    for i, action in enumerate(plan.actions):
        action_name = action.action.name
        params = [p.object().name for p in action.actual_parameters]
        
        print(f"\n--- Step {i+1}/{len(plan.actions)}: Executing {action_name}{params} ---")

        task = None
        #core translation logic: map PDDL actions to PyRoboSim actions
        if action_name == "move":
            # PDDL move parameters: (robot, from_location, to_location)
            target_loc_name = params[2]
            target_location_obj = location_map.get(target_loc_name)
            if not target_location_obj:
                print(f"  [ERROR] Cannot find location: {target_loc_name}")
                break

            nav_pose = None 

            # 1. first check the target location itself for nav_poses
            if target_location_obj.nav_poses:
                nav_pose = target_location_obj.nav_poses[0]
            
            # 2. if no nav_pose, check sub-locations
            elif hasattr(target_location_obj, 'children') and target_location_obj.children:
                for sub_loc in target_location_obj.children:
                    if sub_loc.nav_poses:
                        nav_pose = sub_loc.nav_poses[0]
                        print(f"  Found nav_pose in sub-location: '{sub_loc.name}' of '{target_loc_name}'")
                        break # if found, break the loop
            
            # 3.if found nav_pose
            if nav_pose:
                task = TaskAction("navigate", target_location=nav_pose)
            else:
            # 4.otherwise report error
                print(f"  [ERROR] Target location '{target_loc_name}' and its sub-locations have NO navigation poses defined. Aborting plan.")
                break 

        elif action_name == "pick":
            # PDDL pick parameters: (robot, object, location)
            object_name = params[1]
            target_object_obj = object_map.get(object_name)
            if target_object_obj:
                task = TaskAction("pick", object=target_object_obj)
            else:
                print(f"  [ERROR] Cannot find object: {object_name}")
                break

        elif action_name == "place": 
            target_loc_name = params[2]
            target_location_obj = location_map.get(target_loc_name)
            if target_location_obj:
                task = TaskAction("place", target_location=target_location_obj)
            else:
                print(f"  [ERROR] Cannot find location: {target_loc_name}")
                break
        
        # execute TaskAction
        if task:
            task_plan = TaskPlan(actions=[task]) #let single action into a plan
            result = robot.execute_plan(task_plan)
            
        
print("\nPHASE 4: PLAN EXECUTION FINISHED!")

def create_world_from_yaml(world_file: str) -> World:
    return WorldYamlLoader().from_file(os.path.join(data_folder, world_file))


def explore_world_and_discover_objects(world: World):
    print("PHASE 1: STARTING EXPLORATION...")
    robot = world.robots[0]
    discovered_objects = []

    # 1. create a liste of waypoints to visit (room nav poses or room centers)
    locations_to_visit = [loc for loc in world.locations if loc.name != "world"]
    
    if not locations_to_visit:
        print("No locations found in the world to visit.")
        return {}

    print(f"Generated a plan to visit {len(locations_to_visit)} locations (furniture).")
    # 2. visit each location
    for i, location in enumerate(locations_to_visit):
       nav_pose = location.nav_poses[0] if location.nav_poses else location.pose
       print(f"\n---> Visiting location {i+1}/{len(locations_to_visit)}: '{location.name}' in room '{location.get_room_name()}'")

       #let robot navigate to the location
       action = TaskAction("navigate", target_location=nav_pose)
       plan = TaskPlan(actions=[action])
       print(f"  Executing plan: Navigate to '{location.name}'")
       robot.execute_plan(plan)
       
       print(f"  Navigation SUCCESS. Programmatically querying objects at '{location.name}'...")

       objects_on_location = []
       for obj in world.objects:
            if compare(obj.parent.name,location.name):
                objects_on_location.append(obj)
                discovered_objects.append(obj)
            else:
                continue

       print(objects_on_location)
    print("\n-----------------------------------")
    print("PHASE 1: EXPLORATION COMPLETE!")
    print(f"Discovered a total of {len(discovered_objects)} objects.")
    print("-----------------------------------")
    return discovered_objects


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="Main pyrobosim demo.")
    parser.add_argument(
        "--multirobot",
        action="store_true",
        help="If no YAML file is specified, this option will add "
        "multiple robots to the world defined in this file.",
    )
    parser.add_argument(
        "--world-file",
        default="",
        help="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    parser.add_argument(
        "--partial-obs-objects",
        action="store_true",
        help="If True, robots have partial observability of objects and must detect them.",
    )
    parser.add_argument(
        "--lidar",
        action="store_true",
        help="If True, adds a lidar sensor to the first robot.",
    )
    return parser.parse_args()



if __name__ == "__main__":
    args = parse_args()
    world = create_world(args.multirobot) if args.world_file == "" \
            else create_world_from_yaml(args.world_file)
    discovered_objects = explore_world_and_discover_objects(world)
    print(discovered_objects)

    domain_pddl_file = "domain.pddl"
    problem_pddl_file = "problem.pddl"
    final_plan = solve_with_upf(domain_pddl_file, problem_pddl_file)
    print(final_plan)
    execute_upf_plan(world, final_plan)
    start_gui(world)      # start the GUI and block here
