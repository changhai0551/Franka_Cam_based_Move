import pyrealsense2 as rs
import pdb,time,os,sys
import cv2
from realsense_device_manager import DeviceManager
import numpy as np


# one camera
class Camera():
    def __init__(self):
        resolution_width = 1280  # 640 # 1280 # pixels
        resolution_height = 720  # 480 # 720 # pixels
        frame_rate = 30  # fps
        dispose_frames_for_stablisation = 5  # frames
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, frame_rate)
        rs_config.enable_stream(rs.stream.infrared, 1, resolution_width, resolution_height, rs.format.y8, frame_rate)
        rs_config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, frame_rate)
        self.device_manager = DeviceManager(rs.context(), rs_config)
        self.device_manager.enable_all_devices()
        # print(str(len(self._available_devices)) + " devices have been found")
        # device_manager.load_settings_json('bolt_realsense.json')
        self.device_manager.load_settings_json('config/d435_high_accuracy.json') # d435_medium_density,  d435_high_accuracy, d435_high_density, high_accuracy
        self.cam_num = len(self.device_manager._available_devices)
        ############################

        for frame in range(dispose_frames_for_stablisation):
            # frames = device_manager.poll_frames()
            frames = self.device_manager.poll_aligned_frames()

        if not frames is {}:
            intrinsics_devices = self.device_manager.get_device_intrinsics(frames)
            intrinsics_cameras = {}
            for serial in self.device_manager._available_devices:
                print(serial)
                mtx_camera = [intrinsics_devices[serial][rs.stream.color].width,
                              intrinsics_devices[serial][rs.stream.color].height,
                              intrinsics_devices[serial][rs.stream.color].ppx,
                              intrinsics_devices[serial][rs.stream.color].ppy,
                              intrinsics_devices[serial][rs.stream.color].fx,
                              intrinsics_devices[serial][rs.stream.color].fy
                              ]
                cameraMatrix = np.array(
                    [[mtx_camera[4], 0, mtx_camera[2]], [0, mtx_camera[5], mtx_camera[3]], [0, 0, 1]])
                intrinsics_cameras[serial] = cameraMatrix
                self.camIntrinsics = cameraMatrix   # 1280*720, '819612070850'

    def get_data(self):
        '''
        return color in BGR (cv2), depth (need /1000.)
        :return:
        '''
        t0 = time.time()
        while True:
            # frames = device_manager.poll_frames()
            frames = self.device_manager.poll_aligned_frames()
            if frames is {} or len(frames) != self.cam_num:
                continue
            break
        for serial in self.device_manager._enabled_devices:
            color_img = np.array(frames[serial][rs.stream.color].get_data())
            depth_img = np.array(frames[serial][rs.stream.depth].get_data())

        print('% d camera time : %f ' % (len(self.device_manager._enabled_devices), time.time() - t0))
        # return [color_left,color_right],[depth_left,depth_right]
        return color_img, depth_img


    def getPointCloud(self,color,depth,urPose,roi=[0,0,1280,720]):
        '''
        get point cloud from end-effector
        :param color:
        :param depth:
        :param roi:
        :return:
        '''
        # color, depth = self.get_data()

        camIntrinsics = self.camIntrinsics
        camPose = self.camPose
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        heightIMG = 720
        widthIMG = 1280
        depthImg = depth / 1000.
        [pixX, pixY] = np.meshgrid(np.arange(widthIMG), np.arange(heightIMG))
        camX = (pixX - camIntrinsics[0][2]) * depthImg / camIntrinsics[0][0]
        camY = (pixY - camIntrinsics[1][2]) * depthImg / camIntrinsics[1][1]
        camZ = depthImg
        xmin,ymin,xmax,ymax = roi
        camX = camX[ymin:ymax, xmin:xmax]
        camY = camY[ymin:ymax, xmin:xmax]
        camZ = camZ[ymin:ymax, xmin:xmax]
        rgb = color[ymin:ymax, xmin:xmax]
        rgb = rgb.reshape((-1, 3)) / 255.
        camPts = [camX.reshape(camX.shape + (1,)), camY.reshape(camY.shape + (1,)), camZ.reshape(camZ.shape + (1,))]
        camPts = np.concatenate(camPts, 2)
        camPts = camPts.reshape(
            (camPts.shape[0] * camPts.shape[1], camPts.shape[2]))  # shape = (heightIMG*widthIMG, 3)
        worldPts = np.dot(camPose[:3, :3], camPts.transpose()) + camPose[:3, 3].reshape(3,1)  # shape = (3, heightIMG*widthIMG)
        worldPts = np.dot(urPose[:3, :3], worldPts) + urPose[:3, 3].reshape(3, 1)
        xyz = worldPts.transpose()
        # xyz = xyz[xyz[:, 2] != 0]
        # rgb = rgb[xyz[:, 2] != 0]
        return xyz,rgb



def echo_client(conn,model):
    try:
        while True:
            msg = conn.recv()
            # msg is ['123146',None,image] image is np.array(())
            pdb.set_trace()
            key = msg[0]
            # assert type(key) is str , 'Incorrect data format.'
            print(key)

            colorData, heightMapPosData = model.getInput()
            np.save('data/colordata%0.3d.npy'%int(key), colorData)
            np.save('data/heightmappos%0.3d.npy'%int(key), heightMapPosData)

            msg = [key,True,colorData, heightMapPosData]
            conn.send(msg)
    except EOFError:
        print('Connection closed')

def echo_server(address, authkey, model):
    from multiprocessing.connection import Listener
    import traceback
    serv = Listener(address, authkey=authkey)
    i = 0
    while True:
        try:
            client = serv.accept()
            i+=1
            print(i)
            echo_client(client, model)
        except Exception:
            traceback.print_exc()

if __name__ == "__main__":

    cam = Camera()
    while True:
        color, depth = cam.get_data()
        cv2.namedWindow('depth')
        cv2.imshow('depth', depth)
        cv2.namedWindow('color')
        cv2.imshow('color', color)
        cv2.waitKey(1)
