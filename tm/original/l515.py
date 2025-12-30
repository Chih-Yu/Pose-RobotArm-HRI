## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QThread, pyqtSignal
from robotControl_UI import Ui_Form
import sys
from PyQt5.QtGui import QPixmap, QImage
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time

class l515(threading.Thread):

    def __init__(self):
        # QThread.__init__(self)
        # self.stopped = event
        threading.Thread.__init__(self)
        # self.recording()

        self.depth_img = None
        self.color_img = None

        self.depthViewer = None
        self.colorViewer = None
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.flag = False


      

    
    # def run(self):
    #     while not self.stopped.wait(0.02):
    #         self.stopped.emit()

    # def run(self):
    #     self.recording_without_param()

        
    def recording(self, depth_viewer, color_viewer):
        # Configure depth and color streams
        # self.pipeline = rs.pipeline()
        # config = rs.config()


        try:
            self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

            align_to = rs.stream.color
            align = rs.align(align_to)

            self.pipeline.start(self.config)
            print('mode 1')

        except:
            try:
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

                align_to = rs.stream.color
                align = rs.align(align_to)

                self.pipeline.start(self.config)
                print('mode 2')
            except:
                self.config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
                self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)

                align_to = rs.stream.color
                align = rs.align(align_to)

                self.pipeline.start(self.config)
                print('mode 3')

        

        # Start streaming
        # intr = self.pipeline.get_active_profile().as_video_stream_profile().get_intrinsics()
        # print(intr)
        

        try:
            while self.flag:

                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.first(rs.stream.color)
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Stack both images horizontally
                # images = np.hstack((color_image, depth_colormap))

                # Show images
                self.depth_img = depth_colormap
                # cv2.namedWindow('RealSense_depth', cv2.WINDOW_NORMAL)
                # cv2.resizeWindow('RealSense_depth', (int(depth_colormap.shape[1]/2),int(depth_colormap.shape[0]/2)))
                # cv2.imshow('RealSense_depth', depth_colormap)
                # print('depth ', depth_colormap.shape)

                # show depth on UI
                rgbImg = cv2.cvtColor(self.depth_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImg.shape
                bytesPerLine = ch * w
                qtImg = QImage(rgbImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
                qtImg = qtImg.scaled(864, 486, QtCore.Qt.KeepAspectRatio)
                depth_viewer.setPixmap(QPixmap.fromImage(qtImg))
                

                self.color_img = color_image
                # cv2.namedWindow('RealSense_rgb', cv2.WINDOW_NORMAL)
                # cv2.resizeWindow('RealSense_rgb', (int(color_image.shape[1]/2),int(color_image.shape[0]/2)))
                # cv2.imshow('RealSense_rgb', color_image)
                # print('color ', color_image.shape)

               

                # show RGB on UI
                rgbImg = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImg.shape
                bytesPerLine = ch * w
                qtImg = QImage(rgbImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
                qtImg = qtImg.scaled(864, 486, QtCore.Qt.KeepAspectRatio)
                color_viewer.setPixmap(QPixmap.fromImage(qtImg))

                cv2.waitKey(1)


        finally:

            # Stop streaming
            self.pipeline.stop()

    def get_Img(self):
        dImg = self.depth_img
        rgbImg = self.color_img

        return dImg, rgbImg

    def setShowLabel(self, depth_viewer, color_viewer):
        self.depthViewer = depth_viewer
        self.colorViewer = color_viewer

    



if __name__ == '__main__':
    l = l515()
    # l.recording()
    # l.recording()
    # print(1)