from argparse import ArgumentParser
import numpy as np
import sys
from rfrankx import Affine, JointMotion, Robot, Waypoint, WaypointMotion, MoveWaypointMotion
import rfrankx
import pdb
from transform3d import Transform
# pdb.set_trace()

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()
    # rfrankx.add(1,2)
    # rfrankx.testprint()
    # Connect to the robot
    robot = Robot(args.host, repeat_on_error=False)
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.set_dynamic_rel(0.2)
    # robot.current_pose()
    # np.array(robot.read_once().O_T_EE).reshape(4, 4).T
    pose1 = Waypoint(Affine(0., -0.05, 0, 0., -0., 0.), -0.0380, Waypoint.Relative)
    pdb.set_trace()
    #############################################
    # waypoint = Waypoint(Affine(0.500959, -0.061772, 0.487196, -0.093157, -0.001081, 3.038385), -0.2, Waypoint.Absolute)
    pose1 = Waypoint(Affine(0.403529, 0.024675, 0.529336, 0.022764, -0.770939, 3.050354), 0.34977, Waypoint.Absolute)
    pose2 = Waypoint(Affine(0.404250, 0.190230, 0.403997, 0.493764, -0.472406, 2.765065), 0.8147, Waypoint.Absolute)
    motion_down = WaypointMotion([pose1, pose2])
    #
    # # You can try to block the robot now.
    robot.move(motion_down)
    pdb.set_trace()





    #############################################

    # joint_motion = JointMotion([-1.0, 0, 1.0, -1.6, 0.0, 1.6, -2.5])
    joint2 = [1.182225251800949, -0.5273177195765826, -1.2623061685896755, -2.0766707371159603, -0.2919127081429459, 1.6921455342287042, 0.7338029986078157]
    joint1 = [2.0489815386924866, -0.5843167256389248, -2.0282640731330166, -1.9333003799712163, -0.29245471532707795, 3.009355127943887, 0.5171093538651864]
    # joint_motion = JointMotion(joint1)
    robot.move(JointMotion(joint1))
    robot.move(JointMotion(joint2))
    pdb.set_trace()

    print("init rfmove Model and State")

    # [0.500959, -0.061772, 0.487196, -0.093157, -0.001081, 3.038385]

    # waypoints = Waypoint(np.array([0.50, -0.06, 0.48]), np.array([-0.09, -0.00, 3.03]))
    # waypoints = Waypoint(Affine(0.50, -0.06, 0.48, -0.09, -0.00, 3.03))
    # motion_down_rfmove=MoveWaypointMotion([waypoints])
                                          #Waypoint(np.array([0.5,0.3,0.6]),np.array([0.0,3.14,0])),
                                          #Waypoint(np.array([0.5,0.35,0.6]),np.array([0.0,3.16,0])),
                                          #Waypoint(np.array([0.5,0.2,0.6]),np.array([0.0,3.11,0]))])
    # robot.move(motion_down_rfmove)

    # Define and move forwards
    # motion_down = WaypointMotion([
    #      Waypoint(Affine(0.0, 0.0, -0.12), -0.2, Waypoint.Relative),
    #      Waypoint(Affine(0.1, 0.0, 0.0), 0.0, Waypoint.Relative),
    #      Waypoint(Affine(0.0, 0.4, 0.0, 0.0), 0.0, Waypoint.Relative),
    #  ])
    # motion_down = WaypointMotion([
    #     Waypoint(Affine(0.0, 0.0, 0.12), -0.2, Waypoint.Relative),Waypoint(Affine(0.0, 0.0, 0.12), -0.2, Waypoint.Relative),
    #     Waypoint(Affine(0.0, 0.0, -0.12), -0.2, Waypoint.Relative)])
    waypoint = Waypoint(Affine(0.500959, -0.061772, 0.487196, -0.093157, -0.001081, 3.038385), -0.2, Waypoint.Absolute)
    pose1 = Waypoint(Affine(0.356420, 0.147856, 0.406751, 0.589425, 0.210525, 2.772243), 0.2239, Waypoint.Absolute)
    pose2 = Waypoint(Affine(0.686215, -0.060573, 0.377306, 0.268292, -0.469433, 2.818482), -0.0871, Waypoint.Absolute)
    motion_down = WaypointMotion([
        pose1,pose2])
    #
    # # You can try to block the robot now.
    pdb.set_trace()
    robot.move(motion_down)
