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


class AlanTuring(Robot):
    def __init__(
        self,
        name="robot",
        pose=...,
        radius=0,
        height=0,
        color=...,
        max_linear_velocity=np.inf,
        max_angular_velocity=np.inf,
        max_linear_acceleration=np.inf,
        max_angular_acceleration=np.inf,
        path_planner=None,
        path_executor=None,
        grasp_generator=None,
        sensors=None,
        start_sensor_threads=True,
        partial_obs_objects=False,
        partial_obs_hallways=False,
        action_execution_options=...,
        initial_battery_level=100,
    ):
        super().__init__(
            name,
            pose,
            radius,
            height,
            color,
            max_linear_velocity,
            max_angular_velocity,
            max_linear_acceleration,
            max_angular_acceleration,
            path_planner,
            path_executor,
            grasp_generator,
            sensors,
            start_sensor_threads,
            partial_obs_objects,
            partial_obs_hallways,
            action_execution_options,
            initial_battery_level,
        )
