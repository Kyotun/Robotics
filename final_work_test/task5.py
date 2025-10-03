#!/usr/bin/env python3
"""

This script:
  - builds/loads a PyRoboSim world,
  - loads PDDL domain/problem from files (REQUIRED),
  - solves with a UPF oneshot planner (e.g., pyperplan),
  - executes the plan by navigating the robot room-to-room.

Usage:
  python task.py --domain-file domain.pddl --problem-file problem.pddl
  python task.py --world-file example_world.yaml --domain-file ... --problem-file ...
  python task.py --no-gui --domain-file ... --problem-file ...
  python task.py --engine pyperplan --domain-file ... --problem-file ...
"""

import os
import argparse
import time
from typing import Tuple

# PyRoboSim
from pyrobosim.gui import start_gui

# Unified Planning
import unified_planning as up
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner
from world_helpers import WorldHelper
from cli import parse_args


if __name__ == "__main__":
    args = parse_args()
    domain_file = args.domain_file
    world_helper = WorldHelper(world_file=args.world_file)
    world = world_helper.getWorld

    # Silence UP credits banner (optional)
    up.shortcuts.get_environment().credits_stream = None

    def thread_func():
        #discovered_objects = world_helper.exploreAndDiscover()
        #print(discovered_objects)

        #problem_file = world_helper.generateProblemPDDL(
        #    problem_for_pddl="task5", domain_for_pddl="task5-dynamic"
        # ) #动态生成pddl problem
        

        #problem_file_name = world_helper.writeProblemPDDL(pddl_as_str=problem_file)#将字符串变成一个临时problem文件 便于URF从文件读取
        #URF求解
        problem_file = "problem.pddl"
        plan = world_helper.solveWithUPF(
            domain_pddl=domain_file, problem_pddl=problem_file#problem_file_name
        )
        print(plan)

        #执行计划
        world_helper.executeUPFPlan(plan=plan)

    import threading #GUI 和 线程并发

    threading.Thread(target=thread_func, daemon=True).start()
    start_gui(world)
    print("[DONE] execution finished.")
