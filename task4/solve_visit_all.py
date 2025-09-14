from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader

reader = PDDLReader()
problem = reader.parse_problem("domain_visit_all.pddl", "problem_visit_all.pddl")

with OneshotPlanner(name="pyperplan") as planner:
    result = planner.solve(problem)
    print("Status:", result.status)
    if result.plan is not None:
        print("Plan:")
        for a in result.plan.actions:
            print(a)
