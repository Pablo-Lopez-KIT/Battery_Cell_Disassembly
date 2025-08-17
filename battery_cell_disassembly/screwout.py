import rclpy
from rclpy.node import Node
from battery_cell_disassembly.ur5_trajectory_publisher import URTrajectoryPublisher


class screwout(Node):
    def __init__(self):
        super().__init__('screwout')
        self.get_logger().info("Starting Simulation.")
        self._executor_shutdown = False

        # 🔁 IMPORTANTE: creas aquí el robot (antes estaba dentro de run_sim)
        import os
        import pybullet as p
        import pybullet_industrial as pi
        from ament_index_python.packages import get_package_share_directory

        pkg_path = get_package_share_directory('battery_cell_disassembly')
        robot_path = os.path.join(pkg_path, 'urdf', 'ur10e_1200x600.urdf')
        self.robot1 = None

        # ✅ Ahora que tienes self.robot1, puedes usarlo aquí
        from battery_cell_disassembly.ur5_trajectory_publisher import URTrajectoryPublisher
        self.publisher_node = None

        self.trajectory_points = []

        # Ejecutas la simulación normalmente
        self.timer = self.create_timer(1.0, self.run_sim)


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
                # ✅ Solo una vez por cada segmento completo
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

                # calcula distancia y pasos para velocidad deseada
                dist = np.linalg.norm(end_pos - start_pos)
                speed = speeds_m_per_s[i] if speeds_m_per_s else 0.01  # m/s
                duration = dist / speed                                # segundos
                steps = max(int(duration / dt), 1)                     # nº pasos

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
            
            # Ejecutar trayectoria real
            # Puedes modificar estas variables arriba en run_sim si quieres
            duration_even = 4.0              # duración entre punto par e impar (ej. movimiento)
            duration_odd = 2.0               # duración entre punto impar y el siguiente (ej. espera)
            base_special_durations = {
                2: 1.5,
                3: 0.5,
                4: 3.5,
                6: 2.0
            }
            period = 22
            special_durations = {}
            # Extend to repeat every 20 points
            for base_index, duration in base_special_durations.items():
                for i in range(0, 1000):  # up to 1000 to be safe
                    index = base_index + i * period
                    special_durations[index] = duration

            default_duration = 2.0           # fallback por si algo no encaja
            timings = []
            current_time = 5.0  # empieza en 5 segundos

            for i in range(len(self.trajectory_points)):
                timings.append(current_time)

                # No sumes nada en el último punto
                if i == len(self.trajectory_points) - 1:
                    break

                # Duración específica si está definida
                if i in special_durations:
                    current_time += special_durations[i]
                # Alternancia par/impar
                elif i % 2 == 0:
                    current_time += duration_even
                elif i % 2 == 1:
                    current_time += duration_odd
                else:
                    current_time += default_duration

            print("ur10 moving...")
            print(f"[DEBUG] Trajectory has {len(self.trajectory_points)} points.")
            self.publisher_node.set_trajectory(self.trajectory_points, timings)

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

            # Orientaciones distintas por tramo (n = len(positions) - 1 = 10)
            if use_special_orientation:
                orientations = [
                    [0.70711, 0.00079, 0.00078, 0.70711]
                ] + [orn_start] * (len(positions) - 1)
            else:
                orientations = [orn_start] * len(positions)

            # Colores por tramo
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

            # Velocidades por tramo (m/s)
            speeds = [
                0.04, 0.04, 0.004, 0.04, 0.04,
                0.04, 0.04, 0.04, 0.04, 0.04, 0.04
            ]

            return positions, orientations, colors, speeds

        pysics_client = p.connect(p.GUI, options='--background_color_red=0.8 ' +
                                '--background_color_green=0.9 ' +
                                '--background_color_blue=1')
        # Ruta al archivo URDF
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
            cameraDistance=2,        # How far the camera is from the target
            # Rotation around vertical axis (left-right)
            cameraYaw=50,
            cameraPitch=-30,           # Vertical angle (up-down)
            # Point the camera looks at (e.g., robot base or end-effector)
            cameraTargetPosition=[0, 0, 0]
        )
        self.robot1 = pi.RobotBase(robot_path1, [0, 0, 0], [0, 0, 0, 1])
        self.publisher_node = URTrajectoryPublisher(self.robot1)

        robot1 = self.robot1  # ya no necesitas crearlo de nuevo


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

        static_start = [-0.0124, -0.67985, 0.57985]  # posición inicial
        static_middle_high = [0.07842, -1.00901, 0.1]
        static_middle_low = [0.07842, -1.00901, 0.03783]
        static_middle_offset = [0.07842, -0.84682, 0.03783]

        tool_pos, tool_orn = screwdriver.get_tool_pose()
        print("Start position:", np.round(tool_pos, 5))
        print("Start orientation (quaternion):", np.round(tool_orn, 5))

        screw_positions = [
            [0.80003, -0.32366, -0.01082],
            #[0.80174, -0.01162, -0.02325],
            #[0.80801, 0.37997, -0.02691],
            #[0.84815, 0.38309, -0.02345],
        ]
        # [0.54030, -0.39979, -0.02572],
        # [0.54208, -0.32186, -0.02658],
        # [0.54398, -0.00911, -0.02923],
        # [0.54699, 0.07273, -0.02989],
        # [0.80003, -0.32366, -0.02082],
        # [0.80174, -0.01162, -0.02325],
        # [0.80499, 0.06820, -0.02382],
        # [0.80801, 0.37997, -0.02691],
        # [0.84332, -0.32082, -0.02052],
        # [0.84389, -0.00931, -0.02190],
        # [0.84931, 0.06998, -0.02236],
        # [0.84815, 0.38309, -0.02345],
        # [1.09777, -0.40050, -0.01683],
        # [1.10140, -0.32421, -0.01509],
        # [1.10254, -0.01245, -0.01637],
        # [1.10588, 0.06651, -0.01650],
        # se bajan 4/5 mm en z

        #ros2 topic pub /urscript_interface/script_command std_msgs/msg/String '{data: popup("hello")}' --once
        #ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data: "movel(p[0.4, 0.2, 0.3, 0, 3.14, 0], a=1.2, v=0.25)"}'
        #ros2 topic pub /urscript_interface/script_command std_msgs/msg/String '{data: "vg10_grip(2, 60, 5, False, 0)"}' --once
        #ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 0, state: 1.0}"
        #ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data: "set_digital_out(1, True)"}'
        
        base_position = [-0.15, 1.025, 0.0]

        full_positions = []
        full_orientations = []
        full_colors = []
        full_speeds = []

        # Flag para identificar el primer tornillo
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

        # Lista de puntos y orientaciones
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
        self._executor_shutdown = True  # 🔁 Esto hace que main_combined continúe



def main(args=None):
    rclpy.init(args=args)
    node = screwout()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
