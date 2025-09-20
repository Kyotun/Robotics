import os
import time

# PyRoboSim 
from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
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
        self.checkWorldInit

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
    
    def getRoomNavPose(self, room_name: str) -> Pose:
        """Pick a reasonable navigation target pose for a room."""
        world = self.getWorld
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
        
    def checkWorldInit(self) -> None:
        """If user gave world_file we create it from yaml file, otherwise use createWorld function."""
        self.world = self.createWorld if not self.worldFile else self.createWorldFromYaml


    def createWorldFromYaml(self) -> World:
        return WorldYamlLoader().from_file(os.path.join(self.dataFolder, self.worldFile))


    def createWorld(self) -> World:
        """Create a test world"""
        world = World()

        # Set the location and object metadata
        world.add_metadata(
            locations=[
                os.path.join(self.dataFolder, "example_location_data_furniture.yaml"),
                os.path.join(self.dataFolder, "example_location_data_accessories.yaml"),
            ],
            objects=[
                os.path.join(self.dataFolder, "example_object_data_food.yaml"),
                os.path.join(self.dataFolder, "example_object_data_drink.yaml"),
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
            partial_obs_objects=self.partialObsObjects,
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

        if self.multiRobot:
            robot1 = Robot(
                name="robot1",
                radius=0.08,
                color=(0.8, 0.8, 0),
                path_executor=ConstantVelocityExecutor(),
                grasp_generator=GraspGenerator(grasp_props),
                partial_obs_objects=self.partialObsObjects,
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
                partial_obs_objects=self.partialObsObjects,
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
    

