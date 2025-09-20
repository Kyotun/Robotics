# File: test_script.py
from pyrobosim.core import WorldYamlLoader
import traceback

print("Attempting to load the minimal test world...")
try:
    # 使用我们之前确认过的、适用于您环境的加载方式
    world = WorldYamlLoader().from_file("test_world.yaml")
    
    # 如果成功，打印成功信息和世界内容
    print("\n-------------------------")
    print("SUCCESS: Minimal world loaded successfully!")
    print(f"Rooms found: {[room.name for room in world.rooms]}")
    print(f"Locations found: {[loc.name for loc in world.locations]}")
    print("-------------------------")

except Exception as e:
    # 如果失败，打印精简的错误和完整的追溯信息
    print("\n-------------------------")
    print("ERROR: Failed to load the minimal world.")
    print(f"The specific error was: {e}")
    print("-------------------------")
    traceback.print_exc()