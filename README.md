# Battery Cell Disassembly

This package provides ROS 2 nodes for controlling a **UR10e cobot** and an **OnRobot VGC10 gripper** for the disassembly of EV battery modules.  
It supports both **PyBullet simulation** and **real robot execution** through External Control.

---

## Node Overview

- **Interpreter nodes (recommended for initial use)**  
  - `screwout_interpreter` → runs the unscrewing trajectories in **PyBullet**.  
  - `gripper_interpreter` → runs the VGC10 gripping logic in **PyBullet**.  
  These nodes are intended for simulation only and work reliably in the **Interpreter** setup.  

- **External Control nodes (real robot connection)**  
  - `screwout` → publishes trajectories to the UR10e using External Control.  
  - `gripper` → attempts to control the VGC10 gripper.  
  ⚠️ At this stage, the VGC10 **does not respond** properly when using these nodes with External Control.  

- **Combined execution**  
  - `main_combined` → sequentially launches both screwout and gripper logic.  
    Useful for running complete cycles automatically.

---

## Recommended Usage

- Start with the **Interpreter nodes** (`screwout_interpreter`, `gripper_interpreter`) to verify the trajectories and gripping logic in simulation.  
- Use the **External Control nodes** only if you need to test with the real robot — but note that the VGC10 gripper is not yet functional in this mode.  
- To run both tasks in sequence:  
  ```bash
  ros2 run battery_cell_disassembly main_combined
