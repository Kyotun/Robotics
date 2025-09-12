from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader

reader = PDDLReader()
problem = reader.parse_problem("domain.pddl", "problem.pddl")

with OneshotPlanner(name="pyperplan") as planner:
    result = planner.solve(problem)
    print("Status:", result.status)
    if result.plan is not None:
        for a in result.plan.actions:
            print(a)
