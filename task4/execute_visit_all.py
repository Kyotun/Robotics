from pathlib import Path
from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.planning.actions import TaskAction, TaskPlan
from pyrobosim.gui import start_gui
from pathlib import Path as PPath


# 1) 读取 PDDL 计划
reader = PDDLReader()
problem = reader.parse_problem("domain_visit_all.pddl", "problem_visit_all.pddl")
with OneshotPlanner(name="pyperplan") as planner:
    result = planner.solve(problem)
assert result.plan is not None, "No plan!"

# 2) 把房间名映射到 waypoint 名
room2loc = {
    "office1": "wp_office1_center",
    "office2": "wp_office2_center",
    "kitchen": "wp_kitchen_center",
    "meeting": "wp_meeting_center",   # 如果你用 bathroom，这里换成 "bathroom": "wp_bathroom_center"
}

# 3) 读取 PyRoboSim 世界
world = WorldYamlLoader().from_file(str(PPath("world_fourrooms.yaml")))
robot = world.robots[0]

# 4) 解析 plan，生成导航动作序列
actions = []
for act in result.plan.actions:
    if act.action.name.lower() == "move":
        # 形如 move(r1, office1, office2)
        to_room = act.actual_parameters[2].object().name
        target_loc = room2loc[to_room]
        assert world.get_location_by_name(target_loc) is not None, f"{target_loc} not in world"
        actions.append(TaskAction("navigate", target_location=target_loc))

# 5) 执行计划（GUI 非阻塞启动，边看边跑）
try:
    start_gui(world, blocking=False)
except TypeError:
    import threading
    threading.Thread(target=start_gui, args=(world,), daemon=True).start()

plan = TaskPlan(actions=actions)
robot.task_plan = plan
result_exec, n_done = robot.execute_plan(plan)
print(f"Exec result: {result_exec.status.name}, steps: {n_done}")
