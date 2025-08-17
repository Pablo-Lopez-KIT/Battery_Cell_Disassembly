import rclpy
from battery_cell_disassembly.screwout_interpreter import screwout
from battery_cell_disassembly.gripper_interpreter import gripper

def main(args=None):
    rclpy.init(args=args)

    print("[DEBUG] Opening screwout...")
    screw_node = screwout()

    # No usamos spin, solo esperamos a que termine su lógica
    while not screw_node._executor_shutdown:  # 👈 marca que el nodo ya terminó
        rclpy.spin_once(screw_node, timeout_sec=0.1)

    screw_node.destroy_node()
    print("[DEBUG] Closing screwout.")

    print("[DEBUG] Opening gripper...")
    gripper_node = gripper()
    rclpy.spin(gripper_node)
    print("[DEBUG] Closing gripper.")
    gripper_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
