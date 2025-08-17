# ================================================================
# GRIPPING LOCATION CONFIGURATION
# ================================================================
# This section explains how to define the pick locations and (optionally)
# a tool yaw angle for the VGC10, and how the two point lists map to
# the gripper’s channels.
#
# WHERE TO PUT YOUR POINTS
# ------------------------
# 1) Use the two lists below:
#    - `suction_positions_AB`: points that will grip using BOTH channels (for long pieces).
#    - `suction_positions_B`:  points that will grip using channel B only (for small pieces)
#
# 2) Format of each entry:
#       [x, y, z]                -> position only
#       [x, y, z, yaw_deg]       -> position + extra TCP yaw (degrees)
#
#    Units: x, y, z in meters; yaw in DEGREES (positive per the right-hand rule
#    about the tool’s +Z axis).
#
# EXAMPLES
# --------
#   # Position only
#   [0.74807, -0.16102, -0.00711],
#
#   # Position + a +15° extra yaw about the tool Z during approach/contact
#   [0.75377,  0.22813, -0.00999, 15.0],
#
# Note:
# This example contains the suction coordinates of the battery cell of a Mitsubishi Outlander 2018.

suction_positions_AB = [   
    [0.75107, -0.16102, -0.00711], 
    [0.75777, 0.22313, -0.00999], 
    [0.89108, -0.17299, -0.00558], 
    [0.89749, 0.21632, -0.00470],
]
suction_positions_B = [
    [0.53934, -0.32862, -0.02347], 
    [0.54184, 0.06283, -0.02469], 
    [1.10028, -0.33489, -0.01282], 
    [1.10310, 0.05450, -0.01393],
]


import rclpy
from rclpy.node import Node
from battery_cell_disassembly.ur5_trajectory_publisher import URTrajectoryPublisher


class gripper(Node):
    def __init__(self):
        super().__init__('gripper')
        print("[DEBUG] gripper started")
        self.get_logger().info("Starting Simulation.")

        import os
        import pybullet as p
        import pybullet_industrial as pi
        from ament_index_python.packages import get_package_share_directory

        pkg_path = get_package_share_directory('battery_cell_disassembly')
        robot_path = os.path.join(pkg_path, 'urdf', 'ur10e_1200x600.urdf')
        self.robot1 = None

        from battery_cell_disassembly.ur5_trajectory_publisher import URTrajectoryPublisher
        self.publisher_node = None

        self.trajectory_points = []

        self.timer = self.create_timer(1.0, self.run_sim)

    def send_to_interpreter_joint_space(self, joint_trajectory, timings=None):
        import socket

        urscript_lines = []

        print("\n[DEBUG] Joint trajectory (movej):\n")

        seg_times = None
        if timings and len(timings) >= len(joint_trajectory):
            seg_times = [max(timings[i+1] - timings[i], 0.05) 
                        for i in range(len(joint_trajectory)-1)]

        CYCLE_LEN = 12
        GRIP_AT = 4       
        RELEASE_AT = 7   

        for idx, point in enumerate(joint_trajectory):
            joint_positions = point.positions

            if idx == 0:
                line = f"movej([{', '.join(f'{j:.5f}' for j in joint_positions)}], a=1.2, v=0.25, r=0)"
            else:
                if seg_times is not None:
                    t = seg_times[idx - 1]
                    line = f"movej([{', '.join(f'{j:.5f}' for j in joint_positions)}], t={t:.3f}, r=0)"
                else:
                    line = f"movej([{', '.join(f'{j:.5f}' for j in joint_positions)}], a=1.2, v=0.25, r=0)"

            print(f"{idx+1}: {line}")
            urscript_lines.append(line)

            cycle_idx = idx // CYCLE_LEN
            current_mode = self.mode_by_cycle[cycle_idx] if hasattr(self, "mode_by_cycle") and cycle_idx < len(self.mode_by_cycle) else "AB"

            k = idx % CYCLE_LEN
            if k == GRIP_AT:
                if current_mode == "AB":
                    urscript_lines.append("vg10_grip(2, 80, 0, False, 0)") 
                elif current_mode == "A":
                    urscript_lines.append("vg10_grip(1, 80, 0, False, 0)")  

            if k == RELEASE_AT:
                urscript_lines.append("vg10_release(2, 0, False, 0)")      

        urscript_lines.append("end_interpreter()")

        HOST = "192.168.1.102"
        PORT = 30020
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        for line in urscript_lines:
            sock.sendall((line + "\n").encode("utf-8"))
        sock.close()


    def run_sim(self):
        import os
        import pybullet as p
        import numpy as np
        import pybullet_data
        import pybullet_industrial as pi
        from pybullet_industrial import linear_interpolation
        from pybullet_industrial import circular_interpolation
        from pybullet_industrial import ToolPath
        from pybullet_industrial import EndeffectorTool
        from ament_index_python.packages import get_package_share_directory

        self.timer.cancel()

        self.publisher_node
        self.trajectory_points

        def rotate_about_tool_z(q_base, yaw_deg):
            yaw_rad = np.deg2rad(yaw_deg)
            q_delta = p.getQuaternionFromEuler([0, 0, yaw_rad])  # rotación en Z del TCP
            # Combinar en el marco del TCP: post-multiplicación
            _, q_out = p.multiplyTransforms([0,0,0], q_base, [0,0,0], q_delta)
            return q_out

        def simulate_linear_path(tool: EndeffectorTool, end_point: np.array,
                                end_orientation: np.array,
                                color=None, samples=200):
            start_point, start_orientation = tool.get_tool_pose()
            path = linear_interpolation(start_point=start_point, end_point=end_point,
                                        samples=samples,
                                        start_orientation=start_orientation,
                                        end_orientation=end_orientation)
            if color is not None:
                path.draw(color=color)
            simulate_path(path, tool)



        def simulate_path(path: ToolPath, tool: EndeffectorTool, steps_per_segment=20):
            for target_pos, target_orn, _ in path:
                current_pos, current_orn = tool.get_tool_pose()
                for i in range(steps_per_segment):
                    alpha = i / steps_per_segment
                    interp_pos = (1 - alpha) * np.array(current_pos) + \
                        alpha * np.array(target_pos)
                    interp_orn = p.getQuaternionSlerp(current_orn, target_orn, alpha)
                    tool.set_tool_pose(interp_pos.tolist(), interp_orn)
                    p.stepSimulation()
                if self.publisher_node:
                    point = self.publisher_node.get_point()
                    self.trajectory_points.append(point)


        def create_movement_operations(path: ToolPath, robot: pi.RobotBase):
            elementary_operations = []
            for position, orientation, _ in path:
                elementary_operations.append(
                    lambda i=position, j=orientation: robot.set_endeffector_pose(i, j))
            return elementary_operations


        def simulate_multi_segment_path(tool: EndeffectorTool, waypoints: list, orientations: list,
                                        colors: list = None, speeds_m_per_s: list = None, dt=0.01):
            """
            speeds_m_per_s: lista opcional con velocidades (m/s) para cada tramo
            dt: tiempo simulado por paso de PyBullet (ej. 0.01s)
            """
            self.trajectory_points = []
            for i in range(len(waypoints) - 1):
                start_pos = np.array(waypoints[i])
                end_pos = np.array(waypoints[i + 1])
                start_orn = orientations[i]
                end_orn = orientations[i + 1]
                color = colors[i] if colors else None

                dist = np.linalg.norm(end_pos - start_pos)
                speed = speeds_m_per_s[i] if speeds_m_per_s else 0.01  
                duration = dist / speed                              
                steps = max(int(duration / dt), 1)                 

                path = linear_interpolation(start_point=start_pos, end_point=end_pos,
                                            start_orientation=start_orn, end_orientation=end_orn,
                                            samples=2)
                if color:
                    path.draw(color=color)
                simulate_path(path, tool, steps_per_segment=steps)
            
            print("Simmulation Completed: Press 1 to replicate in ur10...")
            waiting = True
            while waiting:
                keys = p.getKeyboardEvents()
                if ord('1') in keys and keys[ord('1')] & p.KEY_WAS_TRIGGERED:
                    waiting = False
                p.stepSimulation()

            duration_even = 2.0     
            duration_odd  = 1.0     

            CYCLE_LEN = 12
            SPECIAL_IN_CYCLE = {
                4: 3.0,             
                
            }

            start_time = 5.0
            timings = [start_time]

            for i in range(len(self.trajectory_points) - 1):
                k = i % CYCLE_LEN
                if k in SPECIAL_IN_CYCLE:
                    dt = SPECIAL_IN_CYCLE[k]
                elif (i % 2) == 0:
                    dt = duration_even
                else:
                    dt = duration_odd
                timings.append(timings[-1] + dt)

            print("ur10 moving...")
            print(f"[DEBUG] Trajectory has {len(self.trajectory_points)} points.")
            self.send_to_interpreter_joint_space(self.trajectory_points, timings)


        def transform_points_to_relative(global_points, base_position):
            base = np.array(base_position)
            return [np.array(p) + base for p in global_points]


        def generate_unscrew_trajectory(screw_position, static_start, release_position,
                                        orn_start, yaw_deg=None, use_special_orientation=False):
            x, y, z = screw_position
            screw_above = [x, y, 0.15]

            positions = [
                static_start,
                screw_above,
                screw_position,
                screw_above,
                release_position,
                static_start
            ]

            if use_special_orientation:
                base_list = [[0, 1, 0, 0]] + [orn_start] * 5
            else:
                base_list = [orn_start] * 6

            if yaw_deg is None:
                orientations = base_list
            else:
                # aplica yaw en la aproximación y en el contacto (1,2,3), revierte fuera
                orn_rot = rotate_about_tool_z(orn_start, yaw_deg)
                orientations = [
                    base_list[0],   # static_start
                    orn_rot,        # screw_above (aprox)
                    orn_rot,        # screw_position (contacto con yaw)
                    orn_rot,        # screw_above (retirada inicial)
                    base_list[4],   # release_position (vuelve horizontal)
                    base_list[5],   # static_start
                ]

            colors = [
                [1, 0, 0],
                [1, 0.5, 0],
                [1, 1, 0],
                [0, 1, 0],
                [0, 1, 1],
                [0, 0, 1],
            ]

            speeds = [
                0.02, 0.02, 0.02, 0.02, 0.02,
                0.02
            ]

            return positions, orientations, colors, speeds

        pysics_client = p.connect(p.GUI, options='--background_color_red=0.8 ' +
                                '--background_color_green=0.9 ' +
                                '--background_color_blue=1')
        pkg_path = get_package_share_directory('battery_cell_disassembly')
        robot_path1 = os.path.join(pkg_path, 'urdf', 'ur10e_1200x600.urdf')
        tool_path = os.path.join(pkg_path, 'urdf', 'screwdriver.urdf')
        screw_path = os.path.join(pkg_path, 'urdf', 'screw_marker.urdf')
        block_path = os.path.join(pkg_path, 'urdf', 'pilar.urdf')

        p.setPhysicsEngineParameter(numSolverIterations=10000)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        p.resetDebugVisualizerCamera(
            cameraDistance=2,        
            cameraYaw=50,
            cameraPitch=-30,          
            cameraTargetPosition=[0, 0, 0]
        )
        self.robot1 = pi.RobotBase(robot_path1, [0, 0, 0], [0, 0, 0, 1])
        self.publisher_node = URTrajectoryPublisher(self.robot1)

        robot1 = self.robot1 


        screwdriver = EndeffectorTool(tool_path, [0, 0, 0], [0, 0, 0, 1], tcp_frame='tcp2_link')
        screwdriver.couple(robot1, endeffector_name='tool0')
        screw_id = p.loadURDF(screw_path, [-0.15, 1.025, 0.0], useFixedBase=True)

        block_id = p.loadURDF(
            block_path, [-0.15, 1.025, 0.0], useFixedBase=True)

        joint_state = {
            'shoulder_pan_joint': np.pi/2,
            'shoulder_lift_joint': -np.pi/2,
            'elbow_joint': np.pi/3,
            'wrist_1_joint': -np.pi/3,
            'wrist_2_joint': -np.pi/2,
            'wrist_3_joint': np.pi/2
        }
        robot1.set_joint_position(joint_state)
        for _ in range(100):
            p.stepSimulation()
        pt_start, orn_start = robot1.get_endeffector_pose()

        pt_1 = [0.65736, 0.7143, -0.13701]
        orn_1 = p.getQuaternionFromEuler([0, -np.pi, np.pi/2])

        static_start = [0.18705, -0.58833, 0.69751] 
        release_position = [0.0257, -0.44400, 0.25]

        tool_pos, tool_orn = screwdriver.get_tool_pose()
        print("Start position:", np.round(tool_pos, 5))
        print("Start orientation (quaternion):", np.round(tool_orn, 5))

        base_position = [-0.15, 1.025, 0.0]

        full_positions = []
        full_orientations = []
        full_colors = []
        full_speeds = []

        first_screw = True

        for item in suction_positions_AB:
            if len(item) == 4:
                pos = item[:3]
                yaw = item[3]
            else:
                pos = item
                yaw = None  # sin giro adicional

            p_list, o_list, c_list, s_list = generate_unscrew_trajectory(
                pos, static_start, release_position, orn_1,
                yaw_deg=yaw, use_special_orientation=first_screw
            )
            first_screw = False
            full_positions.extend(p_list)
            full_orientations.extend(o_list)
            full_colors.extend(c_list)
            full_speeds.extend(s_list)

        adjusted_B_positions = [[x, y + 0.029, z] for (x, y, z) in suction_positions_B]

        for pos in adjusted_B_positions:
            p_list, o_list, c_list, s_list = generate_unscrew_trajectory(
                pos, static_start, release_position, orn_1, use_special_orientation=False)
            full_positions.extend(p_list)
            full_orientations.extend(o_list)
            full_colors.extend(c_list)
            full_speeds.extend(s_list)

        waypoints = transform_points_to_relative(full_positions, base_position)

        self.mode_by_cycle = ["AB"] * len(suction_positions_AB) + ["A"] * len(suction_positions_B)

        simulate_multi_segment_path(
            screwdriver, waypoints, full_orientations, full_colors, full_speeds)

        for _ in range(100000):
            p.stepSimulation()
        pass

def main(args=None):
    rclpy.init(args=args)
    node = gripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
