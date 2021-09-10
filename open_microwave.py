import pdb
import numpy as np
from robot import *
from camera import Camera
from calibration import calibration
import cv2
from transform3d import Transform
import matplotlib.pyplot as plt
import time

def detectArucoPose(color):
    import cv2.aruco as aruco
    cameraMatrix = np.load('config/franka_mtx.npy')
    distCoeffs = np.array([[0, 0, 0., 0, 0]])
    img = color
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # COLOR_RGB2GRAY
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    rvec, tvec, pose = aruco.estimatePoseSingleMarkers(corners, 0.034, cameraMatrix, distCoeffs)
    QueryImg = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255))
    QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec[0], tvec[0], 0.02)
    rr,_ = cv2.Rodrigues(rvec[0])
    TT = np.identity(4)
    TT[:3,:3] = rr
    TT[:3,3] = tvec[0,0]
    return TT, QueryImg


def getRobotAffine(pose):
    pose = Transform(pose)
    pose = np.hstack((pose.p, pose.euler('ZYX')))
    return pose

def getNewRobotPose(capturePose, markerPose, robotPose):
    '''
    capture the marker at capturePose and return markerPose in cam frame,
    based on offset with markerPose_original,return newgraspPose
    :param capturePose:
    :param markerPose:
    :param robotPose:
    :return:
    '''
    markerPose_original = np.load('config/markerPose.npy')
    camPose = np.load('config/campose20210908_franka.npy')

    currentPose = capturePose
    graspPose = robotPose

    robotcamjoint2graspPose = np.linalg.inv(currentPose).dot(graspPose)
    tempPose = np.linalg.inv(markerPose_original).dot(np.linalg.inv(camPose))
    marker2graspPose = np.dot(tempPose, robotcamjoint2graspPose)  # important

    newgraspPose = np.dot(currentPose, camPose)
    newgraspPose = np.dot(newgraspPose, markerPose)
    newgraspPose = np.dot(newgraspPose, marker2graspPose)
    return newgraspPose

if __name__ == "__main__":
    panda = Panda()
    cam = Camera()
    mtx = np.load('config/franka_mtx.npy')
    camPose = np.load('config/campose20210908_franka.npy')

    markerPose_original = np.load('config/markerPose.npy') # need to reset
    graspPose = np.load('config/graspPose.npy')  # need to reset
    pregraspPose = np.load('config/pregraspPose.npy')  # need to reset
    postgraspPose = np.load('config/postgraspPose.npy')  # need to reset
    # i = 4
    # while i>0:
    #
    # i -= 1
    jointCam = [0.032, -0.668589, 0.0786, -2.90, 0.076280, 3.33, 0.73316]
    try:
        panda.moveJoint(jointCam)
        color, depth = cam.get_data()
        markerPose, color_maker = detectArucoPose(color)
        currentPose = np.array(panda.robot.read_once().O_T_EE).reshape(4, 4).T
        pdb.set_trace()
        newgraspPose = getNewRobotPose(currentPose, markerPose, graspPose)
        graspPose_current = getRobotAffine(newgraspPose)

        newpregraspPose = getNewRobotPose(currentPose, markerPose, pregraspPose)
        newpostgraspPose = getNewRobotPose(currentPose, markerPose, postgraspPose)
        pregraspPose_current = getRobotAffine(newpregraspPose)
        postgraspPose_current = getRobotAffine(newpostgraspPose)

        panda.setGripper(60, 0.1)
        panda.gripper_open()
        # pdb.set_trace()
        # pre grasp joint
        # jointList = [[-0.2419611, -0.0655, -0.037145, -2.4708, -0.4032, 3.7220, 1.013]]
        # panda.moveJoint(jointList[0])
        panda.moveWaypoints([pregraspPose_current.tolist() + [-0.03]])

        panda.moveWaypoints([graspPose_current.tolist() + [-0.03]])
        panda.gripper_close()

        # panda.robot.move(LinearRelativeMotion(Affine(0.0, 0., -0.03)))
        panda.moveWaypoints([postgraspPose_current.tolist() + [-0.03]])
        panda.gripper_open()
        # panda.robot.move(LinearRelativeMotion(Affine(0.0, 0., -0.05)))

        jointHome = [0.1820713962044818, -0.7697010890224522, -0.11439344856073007, -2.5926713192576534,
                     -0.11619233189026515, 2.034433099944058, 1.0307617309937875]
        panda.moveJoint(jointHome)
        time.sleep(1)
        # pdb.set_trace()
    except Errors:
        pdb.set_trace()