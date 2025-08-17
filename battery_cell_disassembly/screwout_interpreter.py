# ================================================================
# SCREW POSITIONS CONFIGURATION
# ================================================================
# This section defines the coordinates of all screws to be processed.
# Coordinates are in meters and follow the [X, Y, Z] format in the
# robot’s reference frame.
#
# To add/remove screws you may edit manually the "screw_positions" list found after this comment.
#
# This example contains the screw positions of the battery cell of a Mitsubishi Outlander 2018:
    #[0.79976, -0.32488, -0.02507],
    #[0.80057, -0.01288, -0.02472],
    #[0.80404, 0.06820, -0.02460],
    #[0.80307, 0.38112, -0.02638],
    #[0.84303, -0.32158, -0.02194],
    #[0.84281, -0.01019, -0.02387],
    #[0.84594, 0.07097, -0.02418],
    #[0.84697, 0.38206, -0.02505],
    #[0.54119, -0.39489, -0.02736],
    #[0.54190, -0.31720, -0.02764],
    #[0.54053, -0.00743, -0.02961],
    #[0.54421, 0.07476, -0.02989],
    #[1.10045, -0.40033, -0.01589],
    #[1.10256, -0.32296, -0.01653],
    #[1.10217, -0.00948, -0.01763],
    #[1.10659, 0.06720, -0.01766],
#
#
# NOTE: Changing these values updates the simulation and real
#       robot trajectory without modifying the rest of the code.
# ================================================================

screw_positions = [
    [0.79976, -0.32488, -0.02507],
    [0.80057, -0.01288, -0.02472],
    [0.80404, 0.06820, -0.02460],
    [0.80307, 0.38112, -0.02638],
    [0.84303, -0.32158, -0.02194],
    [0.84281, -0.01019, -0.02387],
    [0.84594, 0.07097, -0.02418],
    [0.84697, 0.38206, -0.02505],
    [0.54119, -0.39489, -0.02736],
    [0.54190, -0.31720, -0.02764],
    [0.54053, -0.00743, -0.02961],
    [0.54421, 0.07476, -0.02989],
    [1.10045, -0.40033, -0.01589],
    [1.10256, -0.32296, -0.01653],
    [1.10217, -0.00948, -0.01763],
    [1.10659, 0.06720, -0.01766],
]

import rclpy
from rclpy.node import Node
from battery_cell_disassembly.ur5_trajectory_publisher import URTrajectoryPublisher

class screwout(Node):
    def __init__(self):
        super().__init__('screwout')
        self.get_logger().info("Starting Simulation.")
        self._executor_shutdown = False

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

        for idx, point in enumerate(joint_trajectory):
            joint_positions = point.positions

            if idx == 0:
                # First Step
                line = f"movej([{', '.join(f'{j:.5f}' for j in joint_positions)}], a=1.2, v=0.25, r=0)"
            else:
                if seg_times is not None:
                    t = seg_times[idx - 1]
                    line = f"movej([{', '.join(f'{j:.5f}' for j in joint_positions)}], t={t:.3f}, r=0)"
                else:
                    # Fallback:
                    line = f"movej([{', '.join(f'{j:.5f}' for j in joint_positions)}], a=1.2, v=0.25, r=0)"

            print(f"{idx+1}: {line}")
            urscript_lines.append(line)

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
            
            duration_even = 2.0              # move
            duration_odd = 1.0               # wait
            base_special_durations = {
                2: 1.0,
                3: 0.5,
                4: 2.5,
                6: 1.0
            }
            period = 22
            special_durations = {}
            for base_index, duration in base_special_durations.items():
                for i in range(0, 1000):  # up to 1000 to be safe
                    index = base_index + i * period
                    special_durations[index] = duration

            default_duration = 2.0
            timings = []
            current_time = 5.0

            for i in range(len(self.trajectory_points)):
                timings.append(current_time)

                if i == len(self.trajectory_points) - 1:
                    break

                # Special if defined
                if i in special_durations:
                    current_time += special_durations[i]
                # Alternate odd/even
                elif i % 2 == 0:
                    current_time += duration_even
                elif i % 2 == 1:
                    current_time += duration_odd
                else:
                    current_time += default_duration

            print("ur10 moving...")
            print(f"[DEBUG] Trajectory has {len(self.trajectory_points)} points.")
            self.send_to_interpreter_joint_space(self.trajectory_points, timings)


        def transform_points_to_relative(global_points, base_position):
            base = np.array(base_position)
            return [np.array(p) + base for p in global_points]


        def generate_unscrew_trajectory(screw_position, static_start, static_middle_high,
                                        static_middle_low, static_middle_offset,
                                        orn_start, use_special_orientation=False):
            x, y, z = screw_position
            screw_above = [x, y, 0.1]
            screw_offset_z = [x, y, z + 0.02]

            positions = [
                static_start,
                screw_above,
                screw_position,
                screw_offset_z,
                screw_above,
                static_middle_high,
                static_middle_low,
                static_middle_offset,
                static_middle_low,
                static_middle_high,
                static_start
            ]

            # Orientations
            if use_special_orientation:
                orientations = [
                    [0.70711, 0.00079, 0.00078, 0.70711]
                ] + [orn_start] * (len(positions) - 1)
            else:
                orientations = [orn_start] * len(positions)

            # Colors
            colors = [
                [1, 0, 0],
                [1, 0.5, 0],
                [1, 1, 0],
                [0, 1, 0],
                [0, 1, 1],
                [0, 0, 1],
                [0.5, 0, 1],
                [1, 0, 1],
                [0.3, 0.3, 0.3],
                [0, 0, 0],
                [0, 0, 0]
            ]

            # Velocities
            speeds = [
                0.04, 0.04, 0.04, 0.04, 0.04,
                0.04, 0.04, 0.04, 0.04, 0.04, 0.04
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


        screwdriver = EndeffectorTool(tool_path, [0, 0, 0], [0, 0, 0, 1], tcp_frame='tcp_link')
        screwdriver.couple(robot1, endeffector_name='tool0')
        screw_id = p.loadURDF(screw_path, [-0.15, 1.025, 0.0], useFixedBase=True)

        block_id = p.loadURDF(
            block_path, [-0.15, 1.025, 0.0], useFixedBase=True)

        joint_state = {
            'shoulder_pan_joint': np.pi/3,
            'shoulder_lift_joint': -np.pi/2,
            'elbow_joint': np.pi/2,
            'wrist_1_joint': 0,
            'wrist_2_joint': np.pi/3,
            'wrist_3_joint': 0
        }
        robot1.set_joint_position(joint_state)
        for _ in range(100):
            p.stepSimulation()
        pt_start, orn_start = robot1.get_endeffector_pose()

        pt_1 = [0.65736, 0.7143, -0.13701]
        orn_1 = p.getQuaternionFromEuler([0, np.pi/2, np.pi/4])

        static_start = [-0.0124, -0.67985, 0.57985]
        static_middle_high = [0.07642, -1.00901, 0.1]
        static_middle_low = [0.07642, -1.00901, 0.03783]
        static_middle_offset = [0.07642, -0.84682, 0.03783]

        tool_pos, tool_orn = screwdriver.get_tool_pose()
        print("Start position:", np.round(tool_pos, 5))
        print("Start orientation (quaternion):", np.round(tool_orn, 5))

        base_position = [-0.15, 1.025, 0.0]

        full_positions = []
        full_orientations = []
        full_colors = []
        full_speeds = []

        # Identify first screw
        first_screw = True

        for pos in screw_positions:
            p_list, o_list, c_list, s_list = generate_unscrew_trajectory(
                pos, static_start, static_middle_high, static_middle_low,
                static_middle_offset, orn_1, use_special_orientation=first_screw)

            first_screw = False

            full_positions.extend(p_list)
            full_orientations.extend(o_list)
            full_colors.extend(c_list)
            full_speeds.extend(s_list)

        waypoints = transform_points_to_relative(full_positions, base_position)

        simulate_multi_segment_path(
            screwdriver, waypoints, full_orientations, full_colors, full_speeds)
        
        print("Press 0 to finish")

        while True:
            keys = p.getKeyboardEvents()
            if ord('0') in keys and keys[ord('0')] & p.KEY_WAS_TRIGGERED:
                print("Finishing...")
                break
            p.stepSimulation()

        p.disconnect()
        self.get_logger().info("Finished")
        self._executor_shutdown = True  

def main(args=None):
    rclpy.init(args=args)
    node = screwout()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
