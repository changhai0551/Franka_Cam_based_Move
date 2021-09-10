from argparse import ArgumentParser
from time import sleep
from rfrankx import Robot
import numpy as np
import pdb
from transform3d import Transform

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
    args = parser.parse_args()

    robot = Robot(args.host)
    robot.set_default_behavior()

    pose_list = []
    save = True
    # pdb.set_trace()
    while save:
        state = robot.read_once()
        print('\nPose: ', robot.current_pose())
        print('O_TT_E: ', state.O_T_EE)
        print('Joints: ', state.q)
        print('Elbow: ', state.elbow)
        sleep(0.05)
        current_pose = np.array(state.O_T_EE).reshape(4,4).T
        Transform(matrix = current_pose)
        print(current_pose)
        pose_list.append(current_pose)
        pdb.set_trace() # save = False
    np.save('pose_list.npy', np.array(pose_list))

