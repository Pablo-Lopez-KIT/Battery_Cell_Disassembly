# Battery Cell Disassembly

This package provides ROS 2 nodes for controlling a **UR10e cobot** and an **OnRobot VGC10 gripper** for the disassembly of EV battery modules.  
It supports both **PyBullet simulation** and execution on the **real robot**.

---

## Node Overview

- **Interpreter nodes (recommended)**  
  - `screwout_interpreter` → runs the unscrewing trajectories.  
  - `gripper_interpreter` → controls the VGC10 gripper.  
  ✅ These nodes work both in **PyBullet simulation** and on the **real UR10e with the VGC10**.  

- **External Control nodes (not functional with VGC10)**  
  - `screwout` → publishes trajectories to the UR10e.  
  - `gripper` → attempts to control the VGC10.  
  ⚠️ The VGC10 **does not respond** when using these nodes with External Control.  

- **Combined execution**  
  - `main_combined` → sequentially runs screwout and gripper logic.  
    Useful for running a complete cycle automatically.  

---

## Recommended Usage

- Always use the **Interpreter nodes** (`screwout_interpreter`, `gripper_interpreter`) for both simulation and the real robot.  
- Avoid using the **External Control nodes**, since the VGC10 is not functional in that mode.  
- To run both tasks in sequence:  
  ```bash
  ros2 run battery_cell_disassembly main_combined
