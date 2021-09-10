import pdb

from robot import Panda
from camera import CameraL515,Camera
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import os
import os.path as osp

class calibration():
    def __init__(self,pattern_size=(8,6),square_size=15,handeye='EIH'):
        '''

        :param image_list:  image array, num*720*1280*3
        :param pose_list: pose array, num*4*4
        :param pattern_size: calibration pattern size
        :param square_size: calibration pattern square size, 15mm
        :param handeye:
        '''
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.handeye = handeye
        self.pose_list = []
        self.mtx = np.load('config/franka_d415mtx.npy')
        self.init_calib()

    def init_calib(self):
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = self.square_size * np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        for i in range(self.pattern_size[0] * self.pattern_size[1]):
            x, y = self.objp[i, 0], self.objp[i, 1]
            self.objp[i, 0], self.objp[i, 1] = y, x
        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def detectFeature(self,color,show=True):
        img = color
        self.gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(img, self.pattern_size, None,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH)  # + cv2.CALIB_CB_NORMALIZE_IMAGE+ cv2.CALIB_CB_FAST_CHECK)
        if ret == True:
            self.objpoints.append(self.objp)
            # corners2 = corners
            if (cv2.__version__).split('.')[0] == '2':
                # pdb.set_trace()
                cv2.cornerSubPix(self.gray, corners, (5, 5), (-1, -1), self.criteria)
                corners2 = corners
            else:
                corners2 = cv2.cornerSubPix(self.gray, corners, (5, 5), (-1, -1), self.criteria)
            self.imgpoints.append(corners2)
            if show:
                fig, ax = plt.subplots(figsize=(20, 20))
                ax.imshow(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                plt.title('img with feature point')
                for i in range(self.pattern_size[0] * self.pattern_size[1] - 3):
                    ax.plot(corners2[i, 0, 0], corners2[i, 0, 1], 'r+')
                plt.show()

    def rodrigues_trans2tr(self,rvec, tvec):
        r, _ = cv2.Rodrigues(rvec)
        tvec.shape = (3,)
        T = np.identity(4)
        T[0:3, 3] = tvec
        T[0:3, 0:3] = r
        return T


    def cal(self):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], self.mtx, None)
        # Hm2w = []  # robot end-effector pose
        # pdb.set_trace()
        Hg2c = []
        pose_list = np.array(self.pose_list)
        for i in range(len(rvecs)):
            tt = self.rodrigues_trans2tr(rvecs[i], tvecs[i] / 1000.)
            Hg2c.append(tt)
        Hg2c = np.array(Hg2c)
        rot, pos = cv2.calibrateHandEye(pose_list[:, :3, :3], pose_list[:, :3, 3], Hg2c[:, :3, :3], Hg2c[:, :3, 3])
        camT = np.identity(4)
        camT[:3, :3] = rot
        camT[:3, 3] = pos[:,0]
        return camT


if __name__ == "__main__":
    cam = Camera()
    # while True:
    #     color, depth = cam.get_data()
    #     cv2.imshow('color',color)
    #     cv2.waitKey(1)
    panda = Panda()
    jointList = [[-0.16390678621279564, -0.6791972863269473, 0.16378642220873582, -2.494650543919316, 0.10657260429859161, 1.863999169005288, 0.7705911846210559],
                 [-0.747860636764451, -0.8281878829466498, 1.0318797409540117, -2.3930792641855043, 0.2860740681839982, 1.8532774123880598, 0.7908645075159377],
                 [-0.23978973600974976, -0.26543558388649235, -0.282111360968205, -2.3520484751316535, 0.27992621284723274, 1.8615258617603918, 0.22772078000066565],
                 [-0.3200761370750531, -1.266139867682206, 0.1446611231458994, -3.0304218578046225, 0.2911064845522245, 2.036694043362426, 0.457444145068656],
                 [-0.5073305669280521, -0.4878137231584181, 0.4762185306214449, -2.7689264097338833, 0.2874546642712036, 2.35799840742723, 0.51454026906651],]

    calib = calibration()
    # calib_image_root = 'image/20210908/calib'
    pose_list = []
    for i, joint in enumerate(jointList):
        panda.moveJoint(joint)
        current_pose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
        # time.sleep(1)
        color, depth = cam.get_data()
        calib.detectFeature(color)
        calib.pose_list.append(current_pose)

        # os.makedirs(calib_image_root, exist_ok=True)
        # os.makedirs(osp.join(calib_image_root, 'color'), exist_ok=True)
        # os.makedirs(osp.join(calib_image_root, 'depth'), exist_ok=True)
        # cv2.imwrite(osp.join(calib_image_root, 'depth', 'franka_%0.3d.png' % (i + 1)), depth)
        # cv2.imwrite(osp.join(calib_image_root, 'color', 'franka_%0.3d.png' % (i + 1)), color)
        # pose_list.append(current_pose)

    camT = calib.cal()
    # np.save(osp.join(calib_image_root,'pose_list.npy'), np.array(pose_list))
    np.save('config/campose20210909_franka.npy',camT)

    pdb.set_trace()