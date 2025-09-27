import argparse

# ----------------- CLI -----------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Task 4 — Visit-All (UPF + PyRoboSim) with external PDDL files")
    p.add_argument("--world-file", required=True, default="", help="YAML file (under pyrobosim/data) to load a world.")
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
