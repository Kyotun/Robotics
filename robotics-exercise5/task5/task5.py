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
    world_helper = WorldHelper(args.world_file)
    world = world_helper.getWorld

    # Silence UP credits banner (optional)
    up.shortcuts.get_environment().credits_stream = None

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
    world_helper.executeVisitAll(plan_steps)
    print("[DONE] Task 4 execution finished.")
