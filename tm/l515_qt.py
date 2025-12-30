## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from robotControl_UI import Ui_Form
import sys
from PyQt5.QtGui import QPixmap, QImage
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time

class l515(QObject):
    # signal
    finished = pyqtSignal()
    getimage = pyqtSignal()
    update_img = pyqtSignal(np.ndarray, np.ndarray)
    updatetest = pyqtSignal()

    def __init__(self):
        super(l515, self).__init__()


        self.depth_img = None
        self.depth_frame = None
        self.color_img = None
        self.depth_z16 = None
        self.depth_scale = None

        self.depthViewer = None
        self.colorViewer = None
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.depth_filter = rs.colorizer()
        # self.depth_filter.set_option(rs.option.visual_preset, 0) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
        
        # self.depth_filter.set_option(rs.option.max_distance, 16)
        self.intr = None
        self.flag = False
    

        
    def run(self):
        # Configure depth and color streams
        # self.pipeline = rs.pipeline()
        # config = rs.config()
        try:
            self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
          

            align_to = rs.stream.color
            align = rs.align(align_to)

            profile = self.pipeline.start(self.config)
            # profile = pipeline.start(config)
            prof = profile.get_stream(rs.stream.color)
            self.intr = prof.as_video_stream_profile().get_intrinsics()
            print('mode 1')

            sensor_dep = profile.get_device().first_depth_sensor()
            self.depth_scale = sensor_dep.get_depth_scale()
            if sensor_dep.supports(rs.option.depth_units):
                print("Trying to set min_distance")
                sensor_dep.set_option(rs.option.depth_units,0.001)

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
        

        try:
            while self.flag:
                

                try:
                    # Wait for a coherent pair of frames: depth and color
                    frames = self.pipeline.wait_for_frames()
                    aligned_frames = align.process(frames)
                    depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.first(rs.stream.color)
                    if not depth_frame or not color_frame:
                        continue

                    # Convert images to numpy arrays
                    depth_image = np.asanyarray(self.depth_filter.colorize(depth_frame).get_data())
                    self.depth_z16 = np.asanyarray(depth_frame.get_data()) *self.depth_scale * 1000
                    
                    # print('depth_image: ',self.depth_z16)

                    # depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())


                    # show depth on UI
                    self.depth_img = depth_image
                    # print('depth address ', hex(id(self.depth_img)))
                    self.depth_frame = depth_frame
                    

                    self.color_img = color_image

                    # Send image to UI for update
                    self.update_img.emit(self.depth_img, self.color_img)
                    self.updatetest.emit()
                    cv2.waitKey(1)
                
                except Exception as e:
                    # Error occured during execution of the processing block! This because UI thread is busy and cannot update camera image
                    pass
                    # print('L515 catch image error:')
                    # print(e)



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
    
    def get_intrinsic(self):
        return self.intr

    



if __name__ == '__main__':
    l = l515()
    # l.recording()
    l.recording()
    print(1)