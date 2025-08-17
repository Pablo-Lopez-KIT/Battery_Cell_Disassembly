

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_points_from_csv(filename):
    import csv
    points = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for i,row in enumerate(reader):
            if i >0:
                if any(row):
                    point = JointTrajectoryPoint()
                    point.positions = [float(x) for x in row]
                    points.append(point)
    return points


class URTrajectoryPublisher(Node):

    def __init__(self, robot):
        super().__init__('pub_joint_state')
        self.publisher = self.create_publisher(
            JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.robot = robot

        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def get_point(self):
        joint_states = self.robot.get_joint_state()
        print("joint_states keys:", joint_states.keys())
        point = JointTrajectoryPoint()
        point.positions = [joint_states[key]['position']
                           for key in self.joint_names]

        return point

    def set_trajectory(self, points, timings):
        from copy import deepcopy
        import numpy as np
        trajectory_points = []

        for i in range(len(points)):
            new_point = deepcopy(points[i])
            seconds, nanoseconds = self.convert_time(timings[i])
            new_point.time_from_start.sec = seconds
            new_point.time_from_start.nanosec = nanoseconds
            trajectory_points.append(new_point)

        print("[DEBUG] Trajectory to publish:")
        for i, pt in enumerate(trajectory_points):
            print(f"  Point {i}:")
            print(f"    positions: {np.round(pt.positions, 4)}")
            print(f"    time_from_start: {pt.time_from_start.sec}.{pt.time_from_start.nanosec:09d}")

            
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.joint_names
        joint_trajectory.points = trajectory_points
        self.publisher.publish(joint_trajectory)


    @staticmethod
    def convert_time(time_in_seconds):
        seconds = int(time_in_seconds)
        nanoseconds = int((time_in_seconds-seconds)*1e9)
        return seconds, nanoseconds


def main():
    from time import sleep

    import numpy as np
    import pybullet as p
    import rclpy
    from emo_demonstration.base_simulation import setup_base_sim
    rclpy.init()
    robot, endeffector,_ = setup_base_sim(mode="direct")
    publisher_node = URTrajectoryPublisher(robot)

    points = load_points_from_csv('test.csv')
    
    timing= np.arange(len(points))*0.1+2

    while True:
        publisher_node.set_trajectory(points, timing)
        sleep(timing[-1]+2)


if __name__ == "__main__":

    main()
