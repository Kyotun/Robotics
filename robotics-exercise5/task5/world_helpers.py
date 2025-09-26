import os
from pathlib import Path
import time
from typing import Any
import random

# PyRoboSim 
from pyrobosim.core.robot import Robot
from pyrobosim.core.objects import Object
from pyrobosim.core.locations import Location
from pyrobosim.core.world import World
from pyrobosim.core.room import Room
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.a_star import AStarPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.pose import Pose
from pyrobosim.planning.actions import TaskAction, TaskPlan


# Unified Planning
import unified_planning as up
from pyrobosim.utils.general import get_data_folder
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner


class WorldHelper():
    def __init__(self, world_file:str = None,
                 domain_pddl:str = None,
                 problem_pddl:str = None):
        self.data_folder:str = get_data_folder()
        self.world_file:str = world_file
        self.domain_pddl:str = domain_pddl
        self.problem_pddl:str = problem_pddl
        self.world:World = None
        self.multi_robot:bool = False
        self.partial_obs_objects:bool = True
        self.createWorldFromYaml()
        self.ensureRoomNavLocations()

    @property
    def dataFolder(self) -> str:
        return self.data_folder

    @property
    def worldFile(self) -> str:
        return self.world_file

    @property
    def problemPDDL(self) -> str:
        return self.problem_pddl
    
    @property
    def domainPDDL(self) -> str:
        return self.domain_pddl
     
    @property
    def multiRobot(self) -> bool:
        return self.multi_robot
    
    @property
    def partialObsObjects(self) -> bool:
        return self.partial_obs_objects
    
    @property
    def getWorld(self) -> World:
        return self.world
    
    def getLocationOfObject(self, object:Object) -> Location:
        world = self.getWorld
        locations = world.locations
        for location in locations:
            # I don't know if this is the best way to define
            if object.parent.name.startswith(location.name):
                return location
        raise Exception(f"Parent of {object.name} cannot found.")
    
    def getRoomOfLocation(self, location:Location) -> Room:
        world = self.getWorld
        rooms = world.rooms
        for room in rooms:
            if location.parent.name.startswith(room.name):
                return room
        raise Exception(f"Parent of {location.name} cannot found.")


    def getRoomCenter(room: Room) -> Pose:
        """Returns the x and y coordinate of the room."""
        xs = [p[0] for p in room.footprint]
        ys = [p[1] for p in room.footprint]
        return Pose(x=sum(xs)/len(xs), y=sum(ys)/len(ys), yaw=0.0)
    

    def getRoomByName(self, room_name:str) -> Room | None:
        world = self.getWorld
        for room in world.rooms:
            if getattr(room, "name", None) == room_name:
                return room
        return None
    

    def getRoomByCenter(self, name_center_room: str) -> Room | None:
        world = self.getWorld
        center_room_name_list = name_center_room.split("_")
        for room in world.rooms:
            if room.name in center_room_name_list:
                return room


    def getRoomBasePose(self, room:Room) -> Pose:
         # 1) explicit nav pose
        navs = getattr(room, "nav_poses", None)
        if navs:
            return navs[0]
        # 2) centroid fallback
        xs = [p[0] for p in room.footprint]
        ys = [p[1] for p in room.footprint]
        return Pose(x=sum(xs)/len(xs), y=sum(ys)/len(ys), yaw=0.0)


    def getRobot(self, preferred_name: str = "") -> Robot:
        world = self.getWorld
        if preferred_name:
            for robot in world.robots:
                if robot.name == preferred_name:
                    return robot
            raise ValueError(f"Robot '{preferred_name}' not found. Available: {[robot.name for robot in world.robots]}")
        if not world.robots:
            raise RuntimeError("No robots in world. Add one with world.add_robot(...)")
        return world.robots[0]


    def getRoomNavPose(self, room_name: str) -> Pose:
        """Pick a reasonable navigation target pose for a room."""
        world = self.getWorld
        room = self.getRoomByName(room_name)
        if room is None:
            available = [r.name for r in world.rooms]
            raise ValueError(
                f"Room '{room_name}' not found. Available rooms: {available}"
            )
        
            # Prefer explicit nav pose
        nav_poses = getattr(room, "nav_poses", None)
        if nav_poses and len(nav_poses) > 0 and isinstance(nav_poses[0], Pose):
            return nav_poses[0]
    
        # Fallback: compute centroid of footprint
        fp = getattr(room, "footprint", None)
        if fp and len(fp) > 0:
            xs = [p[0] for p in fp]
            ys = [p[1] for p in fp]
            return Pose(x=sum(xs) / len(xs), y=sum(ys) / len(ys), yaw=0.0)

        # Ultimate fallback — should never happen on valid rooms
        # but return a real Pose instead of None to avoid "no goal" warnings
        return Pose(x=0.0, y=0.0, yaw=0.0)


    def addNavLocation(self, room_name: str, nav_name: str, base: Pose) -> Location | None:
        """
        Try to place a small, collision-free nav marker in 'room_name'.
        We reuse existing metadata categories (first 'desk', then 'table'),
        because you already load those in Task 1.
        """
        # radial/off-grid offsets to escape furniture & walls
        radii = [0.0, 0.2, -0.2, 0.35, -0.35, 0.5, -0.5, 0.65, -0.65]
        dirs  = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]
        world = self.getWorld
        for r in radii:
            for dx, dy in dirs:
                pose = Pose(x=base.x + dx*r, y=base.y + dy*r, yaw=0.0)
                loc = world.add_location(category="waypoint", parent=room_name, name=nav_name, pose=pose)
                if loc is not None:
                    return loc
        return None


    def ensureRoomNavLocations(self) -> None:
        """
        Create a named location inside each room to use as a navigation target.
        Reuse any existing location category from your metadata (e.g., 'desk' or 'table').
        """
        world = self.getWorld
        for room in world.rooms:
            nav_name = f"nav_{room.name}"
            # Skip if already present
            if any(getattr(loc, "name", "") == nav_name for loc in world.locations):
                continue
            
            base = self.getRoomBasePose(room)
            loc = self.addNavLocation(room.name, nav_name, base)
            if loc is None:
                print(f"[WARN] Could not place '{nav_name}' (room crowded?). "
                    f"Consider adding room.nav_poses for '{room.name}'.")


    def executeVisitAll(self, plan_steps: list):
        """Map PDDL 'move(my_robot, from, to)' to PyRoboSim navigation."""
        world = self.getWorld
        robot = self.getRobot(world)
        for name, params in plan_steps:
            if name != "move":
                print(f"[WARN] Skipping non-move action: {name}")
                continue
            _, frm, to = params
            print(f"[EXEC] move: {frm} -> {to}")
            self.navigateToRoom(world, robot, to, block=True)


    def navigateToRoom(self,
                        robot: Robot,
                        room_name: str,
                        block: bool = True,
                        dt: float = 0.05,
                        timeout_s: float = 60.0) -> None:
        """Command the robot to navigate to a room's nav pose. Poll the world until idle."""
        world = self.getWorld
        goal_pose = self.getRoomNavPose(room_name)
        if goal_pose is None:
            available = [r.name for r in self.world.rooms]
            raise RuntimeError(
                f"[navigateToRoom] Goal pose is None for room '{room_name}'. "
                f"Available rooms: {available}"
            )
        
        print(f"[EXEC] navigate_to {room_name} -> Pose(x={goal_pose.x:.3f}, y={goal_pose.y:.3f}, yaw={getattr(goal_pose, 'yaw', 0.0)})")

        # Sanity: make sure robot has a path planner
        if getattr(robot, "path_planner", None) is None:
            raise RuntimeError("[navigateToRoom] robot.path_planner is None. Set RRT/PRM/A* before navigating.")
    
        robot.navigate(goal_pose)

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

            time.sleep(dt)

            if time.time() - t0 > timeout_s:
                print(f"[WARN] Timeout navigating to {room_name}")
                break


    def createWorldFromYaml(self) -> World:
        self.world = WorldYamlLoader().from_file(f"./{self.world_file}.yaml")
        return self.world
    

    def exploreAndDiscover(self) -> list[Object]:
        def compare(string1:str, string2:str):
            return string1[:5] == string2[:5]

        print("PHASE 1: STARTING EXPLORATION...")
        world = self.getWorld
        robot = world.robots[0]
        discovered_objects = []

        # Create a list of waypoints to visit (room nav poses or room centers)
        locations_to_visit = [loc for loc in world.locations if loc.name != "world"]
        
        if not locations_to_visit:
            print("No locations found in the world to visit.")
            return {}

        print(f"Generated a plan to visit {len(locations_to_visit)} locations.")

        # Visit each location
        for i, location in enumerate(locations_to_visit):
            print(f"\n---> Visiting location {i+1}/{len(locations_to_visit)}: '{location.name}' in room '{location.get_room_name()}'")

            # Let robot navigate to the location
            action = TaskAction("navigate", target_location=location.name)
            plan = TaskPlan(actions=[action])
            print(f"Executing plan: Navigate to '{location.name}'")
            robot.execute_plan(plan)
            
            print(f"Navigation success. Programmatically querying items at '{location.name}'...")

            objects_on_location = []
            for obj in world.objects:
                    if compare(obj.parent.name, location.name):
                        objects_on_location.append(obj)
                        discovered_objects.append(obj)
                    else:
                        continue

            print(f"{location} has: {objects_on_location}")

        print("\n-----------------------------------")
        print("PHASE 1: EXPLORATION COMPLETE!")
        print(f"Discovered a total of {len(discovered_objects)} items.")
        print("-----------------------------------")
        return discovered_objects
    

    def solveWithUPF(self, domain_pddl:str, problem_pddl:str) -> Any | None:
        print("PHASE 2: SOLVING PROBLEM WITH UPF...")
        reader = PDDLReader()
        problem = reader.parse_problem(domain_pddl, problem_pddl)

        with OneshotPlanner(problem_kind=problem.kind) as planner:
            result = planner.solve(problem)
            if result.plan:
                print("UPF found a plan!")
                for action in result.plan.actions:
                    print(f"  - {action}")
                return result.plan
            print("UPF could not find a plan.")
            return None
    
    def executeUPFPlan(self, plan: Any | None) -> None:
        if not plan:
            print("No plan to execute.")
            return
        print("\nPHASE 3: EXECUTING THE FINAL PLAN...")
        world = self.getWorld
        robot = world.robots[0]

        # Save the locations and objects as dict for speed up
        location_map = {loc.name: loc for loc in world.locations}
        object_map = {obj.name: obj for obj in world.objects}

        # Iterate through each action in the UPF plan
        for i, action in enumerate(plan.actions):
            action_name = action.action.name
            params = [parameter.object().name for parameter in action.actual_parameters]
            
            print(f"\n--- Step {i+1}/{len(plan.actions)}: Executing {action_name}{params} ---")

            task = None
            # Core translation logic: map PDDL actions to PyRoboSim actions
            if action_name == "move":
                # PDDL move parameters: (robot, from_location, to_location)
                target_loc_name = params[2]
                target_location_obj = location_map.get(target_loc_name)
                if not target_location_obj:
                    print(f"[ERROR] Cannot find location: {target_loc_name}")
                    break
                task = TaskAction("navigate", target_location=target_location_obj)

            elif action_name == "pick":
                # PDDL pick parameters: (robot, object, location)
                object_name = params[1]
                target_object_obj = object_map.get(object_name)
                if not target_object_obj:
                    print(f"[ERROR] Cannot find item: {object_name}")
                    break
                task = TaskAction("pick", object=target_object_obj)

            elif action_name == "place": 
                target_loc_name = params[2]
                target_location_obj = location_map.get(target_loc_name)
                if target_location_obj:
                    print(f"[ERROR] Cannot find location: {target_loc_name}")
                    break
                task = TaskAction("place", target_location=target_location_obj)
            
            # Execute if task found
            if task:
                task_plan = TaskPlan(actions=[task]) #let single action into a plan
                result = robot.execute_plan(task_plan)
        print("\nPHASE 4: PLAN EXECUTION FINISHED!")

    
    def getRandomLocation(self) -> Location:
        world = self.getWorld
        return random.choice(world.get_locations())

    def generateProblemPDDL(self,
                            problem_for_pddl:str,
                            domain_for_pddl:str) -> str:
        world = self.getWorld
        robots = world.robots
        rooms = world.rooms
        locations = world.locations
        objects = world.objects
        hallways = world.hallways
        first_robot = robots[0]
        problem_for_pddl = problem_for_pddl
        domain_for_pddl = domain_for_pddl
        objects_for_pddl = "EMPTY"

        def generateObjects() -> str:
            if len(robots) > 1:
                raise NotImplementedError
            objects_str = ""
            objects_str += f"{first_robot.name} - robot\n"
            for room in rooms:    
                objects_str += f"{room.name} "
            objects_str += "- room\n"
            for location in locations:
                objects_str += f"{location.name} "
            objects_str += "- location\n"
            for obj in objects:
                objects_str += f"{obj.name} "
            objects_str += "- item\n" 
            return objects_str
        objects_for_pddl = generateObjects()

        def generateInit() -> str:
            loc_of_robot = self.getRoomByCenter(first_robot.location.name).name
            init_str = f"\n(at {first_robot.name} {loc_of_robot})\n"
            init_str += f"(visited {loc_of_robot})\n"
            init_str += f"(handempty {first_robot.name})\n"

            # Where are the locations?
            for location in locations:
                init_str += f"(locationof {location.name} {self.getRoomOfLocation(location=location).name})\n"
            
            # Where are the objects?
            for object in objects:
                init_str += f"(on {object.name} {self.getLocationOfObject(object=object).name})\n"
            
            # Connectivity
            for hallway in hallways:
                init_str += f"(connected {hallway.room_start.name} {hallway.room_end.name}) (connected {hallway.room_end.name} {hallway.room_start.name})\n"
            return init_str
        init_for_pddl = generateInit()

        def generateGoal() -> str:
            target_location = self.getRandomLocation()
            target_str = f"\n(and\n"
            for object in objects:
                target_str += f"(on {object.name} {target_location.name})\n"
            target_str += ")"
            return target_str
        goal_for_pddl = generateGoal()

        return f"(define (problem {problem_for_pddl})\n" \
                f"  (:domain {domain_for_pddl})\n" \
                "   (:objects\n" \
                f"       {objects_for_pddl}" \
                "   )\n" \
                "   (:init" \
                f"       {init_for_pddl}" \
                "   )\n" \
                "   (:goal" \
                f"       {goal_for_pddl}" \
                "   )\n" \
                ")"

    def writeProblemPDDL(self, pddl_as_str:str) -> str:
        if not isinstance(pddl_as_str, str):
            raise TypeError({pddl_as_str}, " should be str.")
        
        file_name = "task5_problem.pddl"
        with open(f"{file_name}", "w") as text_file:
            text_file.write(pddl_as_str)
        return file_name
        



