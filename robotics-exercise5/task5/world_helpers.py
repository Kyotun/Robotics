import os
from pathlib import Path
import time

# PyRoboSim 
from pyrobosim.core.robot import Robot
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

# Unified Planning
from pyrobosim.utils.general import get_data_folder


class WorldHelper():
    def __init__(self, world_file:str = None):
        self.data_folder: str = get_data_folder()
        self.world_file: str = world_file
        self.world: World = None
        self.multi_robot: bool = False
        self.partial_obs_objects: bool = False
        self.createWorldFromYaml()
        self.ensureRoomNavLocations()

    @property
    def dataFolder(self) -> str:
        return self.data_folder

    @property
    def worldFile(self) -> str:
        return self.world_file

    @property
    def multiRobot(self) -> bool:
        return self.multi_robot
    
    @property
    def partialObsObjects(self) -> bool:
        return self.partial_obs_objects
    
    @property
    def getWorld(self) -> World:
        return self.world
    
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