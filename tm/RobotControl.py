#!/usr/bin/env python

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtCore import QMutex, QObject, pyqtSlot, QThreadPool, QRunnable
from PyQt5.QtWidgets import QApplication

from robotControl_UI import Ui_Form
import sys
import os

from PyQt5.QtGui import QPixmap, QImage
import ctypes
from ctypes import cdll
import socket
import numpy as np
import cv2
import time


# from Robot_Control import RobotControl
import RobotControl_func

import warnings
warnings.simplefilter("ignore", UserWarning)
sys.coinit_flags = 2
import l515_qt
import pyrealsense2 as rs

import EIH_calib
import Epson_EIH

import platform

import rclpy
from rclpy.node import Node
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../../', 'tm_msgs/msg'))
# print(sys.path)
# from tm_msgs import msg
from tm_msgs.msg import *
from tm_msgs.srv import *

from detectmarker import *



class getfeedbacklSubscriber(Node):

    def __init__(self):
        super().__init__('getfeedback_subscriber')
        self.subscription = self.create_subscription(
            FeedbackState,
            'feedback_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pos = None

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # print(msg.joint_pos[0])
        # print(msg.joint_pos[1])
        # print(msg.joint_pos[2])
        # print(msg.joint_pos[3])
        # print(msg.joint_pos[4])
        # print(msg.joint_pos[5])
        self.pos = msg

class AppWindow(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.show()

        self.thread = QThread()
        self.LiDar = l515_qt.l515()

        self.ui.textEdit_x.setText(str(376.0587))
        self.ui.textEdit_y.setText(str(-436.6104))
        self.ui.textEdit_z.setText(str(548.17))
        self.ui.textEdit_u.setText(str(179.98))
        self.ui.textEdit_v.setText(str(-0.04045901))
        self.ui.textEdit_w.setText(str(-135.05))

        self.robot = RobotControl_func.RobotControl_Func()

        self.EIH = EIH_calib.calib()

        self.xPos = -1
        self.yPos = -1
        self.distance = -1
        self.calibEstimatedPos = None
        self.west_intr = None
        self.intr = None

        fs = cv2.FileStorage( 'Extrinsic.txt', cv2.FILE_STORAGE_READ )
        fn = fs.getNode("Camera2Gripper")
        Camera2Gripper = fn.mat()
        self.c2g = Camera2Gripper
        fn = fs.getNode("distortion")
        d = fn.mat()
        self.dist = d
        fn = fs.getNode("intrinsic")
        self.mtx = fn.mat()
        
        self.tmp = 2000

        self.label_1 = QtWidgets.QLabel(self)
        # self.label_1.show()
        self.label_1.move(0, 0)
        self.label_1.resize(6, 6)
        self.label_1.setStyleSheet('border: 3px solid blue; border-radius: 3px;')

        self.ui.pushButton_camera_on.clicked.connect(self.turn_on_camera)
        self.ui.pushButton_saveImg.clicked.connect(self.save_currImg)
        # self.ui.pushButton_close_camera.clicked.connect(self.closeCamera)

        self.ui.pushButton_close_camera.clicked.connect(self.moveback)

        self.ui.pushButton_autoCalib.clicked.connect(self.auto_calib)
        # self.ui.pushButton_CalibTest.clicked.connect(self.calib_test)
        self.ui.pushButton_CalibTest.clicked.connect(self.goDetectMarker_2_pick)
        self.ui.pushButton_clearPoint.clicked.connect(self.clearPoint)
        # self.ui.pushButton_Estimated.clicked.connect(self.go_calibEstimatedPos)

        self.ui.pushButton_Estimated.clicked.connect(self.goDetectMarker_2_place)

        self.ui.pushButton_getPos.clicked.connect(self.get_TMPos_ui)
        self.ui.pushButton_setPos.clicked.connect(self.set_TMPos_by_ui)
        self.ui.pushButton_Init_Pos.clicked.connect(self.set_initPos)


        self.ui.label_RGB_viewer.mousePressEvent = self.getmousePos

        rclpy.init(args=None)
        self.minimal_subscriber = getfeedbacklSubscriber()

        self.ask_sta = None
        self.t = None
        self.count = 0

        self.framecoordinate = None





        # initialization

        

        

    def get_TMPos_ui(self):
        # listen to 'feedback_states' topic         
        
        rclpy.spin_once(self.minimal_subscriber)
        data = self.minimal_subscriber.pos
        print(data.tool_pose)
        current_pos = list(data.tool_pose)
        current_pos[0] = current_pos[0] * 1000
        current_pos[1] = current_pos[1] * 1000
        current_pos[2] = current_pos[2] * 1000
        current_pos[3] = current_pos[3] * 180 / np.pi
        current_pos[4] = current_pos[4] * 180 / np.pi
        current_pos[5] = current_pos[5] * 180 / np.pi
        # print(self.robot)

        self.ui.textEdit_x.setText(str(current_pos[0]))
        self.ui.textEdit_y.setText(str(current_pos[1]))
        self.ui.textEdit_z.setText(str(current_pos[2]))
        self.ui.textEdit_u.setText(str(current_pos[3]))
        self.ui.textEdit_v.setText(str(current_pos[4]))
        self.ui.textEdit_w.setText(str(current_pos[5]))



    
    def set_TMPos_by_ui(self):
        x = float(self.ui.textEdit_x.toPlainText())
        y = float(self.ui.textEdit_y.toPlainText())
        z = float(self.ui.textEdit_z.toPlainText())
        u = float(self.ui.textEdit_u.toPlainText())
        v = float(self.ui.textEdit_v.toPlainText())
        w = float(self.ui.textEdit_w.toPlainText())

        self.robot.set_TMPos([x, y, z, u, v, w])
        


    def set_initPos(self):
        x = 376.0587
        y = -436.6104
        z = 548.17
        u = 179.98
        v = -0.04045901
        w = -135.05

        self.robot.set_TMPos([x, y, z, u, v, w])


    def update_UI_img(self, depth_image, color_image):
        rgbImg = depth_image
        h, w, ch = rgbImg.shape
        bytesPerLine = ch * w
        qtImg = QImage(rgbImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
        qtImg = qtImg.scaled(864, 486, QtCore.Qt.KeepAspectRatio)
        self.ui.label_Depth_viewer.setPixmap(QPixmap.fromImage(qtImg))

        rgbImg = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgbImg.shape
        bytesPerLine = ch * w
        qtImg = QImage(rgbImg.data, w, h, bytesPerLine, QImage.Format_RGB888)
        qtImg = qtImg.scaled(864, 486, QtCore.Qt.KeepAspectRatio)
        self.ui.label_RGB_viewer.setPixmap(QPixmap.fromImage(qtImg))

    def turn_on_camera(self):
        self.LiDar.moveToThread(self.thread)
        self.thread.started.connect(self.LiDar.run)
        self.LiDar.finished.connect(self.thread.quit)
        self.LiDar.update_img.connect(self.update_UI_img)

        # self.LiDar.updatetest.connect(self.update_curpos)
        
        self.LiDar.finished.connect(self.LiDar.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
       
        # self.LiDar.progress.connect(self.reportProgress)
        self.LiDar.flag = True
        self.LiDar.setShowLabel(self.ui.label_Depth_viewer, self.ui.label_RGB_viewer)
        self.thread.start()

    def save_currImg(self):
        tmp_color = self.LiDar.color_img
        tmp_depth = self.LiDar.depth_img
        name = 'source/EIH/color_' + str(self.tmp) + '.png'
        cv2.imwrite(name, tmp_color)
        name = 'source/EIH/depth_' + str(self.tmp) + '.png'
        cv2.imwrite(name, tmp_depth)
        self.tmp += 1
        # name = 'source/color.png'
        # cv2.imwrite(name, tmp_color)
        # name = 'source/depth.png'
        # cv2.imwrite(name, tmp_depth)
        # cv2.imshow('color', self.LiDar.color_img)
        # cv2.imshow('depth', self.LiDar.depth_img)
        # cv2.waitKey(1)

    def closeEvent(self, event):
        # num_threads = threading.activeCount()
        # print(num_threads)
        self.LiDar.flag = False
        print('close')

    def closeCamera(self):
        self.LiDar.flag = False
        self.ui.label_Depth_viewer.clear()
        self.ui.label_RGB_viewer.clear()

    def getDepth(self, x, y):
        depth = self.LiDar.depth_frame.as_depth_frame().get_distance(int(x), int(y))*1000
        return depth


    # def on_worker_done(self, threadDone):
    #     print(threadDone)
    #     self.threadDone = threadDone


    
    def auto_calib(self):
        # self.robot.set_TMPos(self.EIH.pos[self.count])
        # self.count += 1

        tmp_pos = self.robot.get_TMPos()
        print(tmp_pos)
        i = 0

        # while True:
        #     if(i == 0):
        #         tmp_pos = self.get_TMPos_new()
        #         tmp_pos1 = tmp_pos
        #         tmp_pos1[2] = tmp_pos1[2] + 20
        #         print(tmp_pos1)
        #         self.robot.set_TMPos(tmp_pos1)
        #         i = 1
        #     else:
        #         tmp_pos = self.get_TMPos_new()
        #         tmp_pos1 = tmp_pos
        #         tmp_pos1[2] = tmp_pos1[2] - 20
        #         print(tmp_pos1)
        #         self.robot.set_TMPos(tmp_pos1)
        #         i = 0

        for p in self.EIH.pos:
            print(p)
            self.robot.set_TMPos(p)
            self.save_currImg()


        

    def getmousePos(self, event):
        print(event.x() * 1920 / 864)
        print(event.y() * 1080 / 486)
        self.xPos = event.x() * 1920 / 864
        self.yPos = event.y() * 1080 / 486

        # self.dist = self.dist[0]
        # print(self.dist)
        # print(self.mtx)

        # x2 = self.xPos ** 2
        # y2 = self.yPos ** 2
        # r2 = x2 + y2
        # _2xy = 2 * self.xPos * self.yPos
        # kr = (1 + ((self.dist[4]*r2 + self.dist[1])*r2 + self.dist[0])*r2)
        # u = self.mtx[0][0]*(self.xPos*kr + self.dist[2]*_2xy + self.dist[3]*(r2 + 2*x2)) + self.mtx[0][2]
        # v = self.mtx[1][1]*(self.yPos*kr + self.dist[2]*(r2 + 2*y2) + self.dist[3]*_2xy) + self.mtx[1][2]

        # self.xPos = u
        # self.yPos = v
        # print(self.xPos, self.yPos)

        tmp_depth = self.LiDar.depth_img
        self.distance = self.LiDar.depth_frame.as_depth_frame().get_distance(int(self.xPos), int(self.yPos))
        print(self.distance)
        self.label_1.move(int(event.x() + 140 - 5), int(event.y() + 30 - 5))
        self.label_1.show()
        # self.label_1.hide()

       
        
        
    def clearPoint(self):
        self.label_1.hide()
        self.xPos = None
        self.yPos = None

    def get_L515_intrinsic(self):
        self.intr = self.LiDar.get_intrinsic()


    def calib_test(self):
        self.west_intr, self.c2g, P2C, g2b = Epson_EIH.EIHCali()
        self.get_L515_intrinsic()
        print(type(self.intr))
        print(type(rs.rs2_deproject_pixel_to_point( self.intr, [float(self.xPos), float(self.yPos)], float(self.distance))))
        print(rs.rs2_deproject_pixel_to_point( self.intr, [float(self.xPos), float(self.yPos)], float(self.distance)))

        # pos[2] = self.distance * 1000
        # self.calibEstimatedPos = pos
        # print(pos)
    
    def go_calibEstimatedPos(self):
        # print(rs.rs2_deproject_pixel_to_point(self.intric, [self.xPos, self.yPos], self.distance))
        # print('calib')
        # print(self.calibEstimatedPos[2])
        # x, y, z, u, v, w = self.robot.get_currPos()

        print(self.xPos, self.yPos)

        xPos = self.xPos
        yPos = self.yPos
        zPos = self.LiDar.depth_frame.as_depth_frame().get_distance(int(xPos), int(yPos))
        print(self.c2g)

        self.get_L515_intrinsic()

        # LiDar function transform 2D(u, v, d) to 3D(X, Y, Z) 
        # ==========================================================
        pos_rs = rs.rs2_deproject_pixel_to_point( self.intr, [float(xPos), float(yPos)], float(zPos))

        # init position
        x = 376.0587
        y = -436.6104
        z = 548.17
        u = 179.98
        v = -0.04045901
        w = -135.05
        print(u)

        RvecsB = Epson_EIH.RotationTrans([float(u), float(v), float(w)])
        # print("Rvecs", i, RvecsB)
        # print(CaliPos[i, 3], ' ', CaliPos[i, 4], ' ', CaliPos[i, 5])
        TvecsB = [[float(x)], [float(y)], [float(z)]]
        # print(TvecsB)
        Gripper2Base = np.r_[np.c_[RvecsB, TvecsB], [[0, 0, 0, 1]]]
        print(Gripper2Base)
        Camera2Base = np.dot(Gripper2Base, self.c2g)
        print(Camera2Base)
        print(Camera2Base[:3, 3])


        Object2Camera = [pos_rs[0]*1000, pos_rs[1]*1000, pos_rs[2]*1000, 1]
        print(Object2Camera)
        
        object2Base = np.dot(Camera2Base, Object2Camera)
        print(object2Base)

        # pos = object2Base[:2]
        print(pos_rs)
        # ==========================================================
        pos_plane = Epson_EIH.EstimateCoord(xPos, yPos, zPos*1000)
        print(pos_plane)


        # add some magical number
        x = pos_plane[0] - 8.5
        y = pos_plane[1] + 8.5
        z = 670.8
        u = 179.98
        v = -0.04045901
        w = -135.05

        self.robot.set_TMPos([x, y, z, u, v, w])

    

    def goDetectMarker_4(self):
        pos=np.array([685.153, 164.948, 672.308, -165.662, -3.228, -44.487])
        id_order = np.array([[1],[2],[3],[4]])


        # move to capture marker
        self.robot.set_TMPos(pos)

        img = np.copy(self.LiDar.color_img)
        M2C, markerCorners, markerCenters = calM2C(img, self.mtx, id_order)
        # tmp_depth = self.LiDar.depth_imgP2B = np.dot(F2B,P2F)mg, self.mtx, id_order)
        print('markerCenters',markerCenters)

        M2B_R=[]
        M2B_T=[]
        for i in range(4):
            depth = self.LiDar.depth_frame.as_depth_frame().get_distance(int(markerCenters[i][0]), int(markerCenters[i][1]))*1000
            M2B_R_tmp, M2B_T_tmp=calM2B(pos[3:], pos[:3], self.c2g, M2C[i], depth)
            M2B_R.append(M2B_R_tmp)
            M2B_T.append(M2B_T_tmp)

        print(M2B_R)
        for i in range(4):
            print('Marker', i)
            print(M2B_T[i])

        F2B, pos = calM2B_R_4(M2B_T)

        pos[2]=pos[2]+400

        print('F2B',F2B)
        print('pos',pos)

        # self.robot.set_TMPos(pos)


    def goDetectMarker_2_pick(self):
        pos0 = np.array([280, -25, 405, -125, -0.02, -45])
        pos = np.copy(pos0)
        id_order = np.array([[1], [2]])

        # move to capture marker
        self.robot.set_TMPos(pos)

        img = np.copy(self.LiDar.color_img)

        M2C, markerCorners, markerCenters = calM2C(img, self.mtx, id_order)
        print('markerCenters', markerCenters)

        M2B_R = []
        M2B_T = []
        for i in range(2):
            depth = 0
            for j in range(5):
                d = self.getDepth(markerCenters[i][0], markerCenters[i][1])
                depth += d
                print('depth', j, d)
                cv2.waitKey(100)
            depth = depth/5
            M2B_R_tmp, M2B_T_tmp = calM2B(pos[3:], pos[:3], self.c2g, M2C[i], depth)
            M2B_R.append(M2B_R_tmp)
            M2B_T.append(M2B_T_tmp)

        print(M2B_R)
        for i in range(2):
            print('Marker', i)
            print(M2B_T[i])

        # midpoint
        # self.robot.set_TMPos(pos0 + np.array([0,0,100,0,0,0]))



        F2B, pos = calM2B_R_2(M2B_T)
        print('F2B', F2B)


        # pos = np.array([340.897, 82.011, 394.137, -88.839, -0.159, -46.019])
        # pos = np.array([359.510,  67.175,  394.137, -88.839, -0.159, -46.019])
        # self.robot.set_TMPos(pos)

        
        P2F = np.array([[1,0,0,30],
                        [0,1,0,-358],
                        [0,0,1,-380],
                        [0,0,0,1]])

        P2B = np.dot(F2B,P2F)
        

        pos = rt2xyzuvw(P2B)      
        print('pos', pos)

        self.framecoordinate=np.copy(pos)

        self.robot.set_TMPos(pos)


        P2F = np.array([[1,0,0,31],
                        [0,1,0,-358],
                        [0,0,1,-52],
                        [0,0,0,1]])

        P2B = np.dot(F2B,P2F)

        pos = rt2xyzuvw(P2B)      
        print('pos', pos)
        
        self.robot.set_TMPos(pos, line=True)
        # lift
        self.robot.set_TMPos(pos+np.array([0, 0, 150,0, 0, 0]),line=True)
        # move above AMM 
        self.robot.set_TMPos(np.array([491.040, -267.207, 425.707, -90.001, 0.001, 135]))
        # place on AMM
        self.robot.set_TMPos(np.array([491.040, -267.207, 325.707, -90.001, 0.001, 135]),line=True)
        # back
        self.robot.set_TMPos(np.array([643.039, -115.206, 325.707, -90.001, 0.001, 135]),line=True)
        # up
        self.robot.set_TMPos(np.array([643.039, -115.206, 625.707, -90.001, 0.001, 135]),line=True)
        # midpoint
        self.robot.set_TMPos(np.array([643.039, -115.206, 625.707, -90.001, 0.001, -135]))



    def goDetectMarker_2_place(self):
        pos0 = np.array([280, -25, 405, -125, -0.02, -45])
        pos = np.copy(pos0)
        id_order = np.array([[1], [2]])

        # move to capture marker
        self.robot.set_TMPos(pos)

        img = np.copy(self.LiDar.color_img)

        M2C, markerCorners, markerCenters = calM2C(img, self.mtx, id_order)
        print('markerCenters', markerCenters)

        M2B_R = []
        M2B_T = []
        for i in range(2):
            depth = 0
            for j in range(5):
                d = self.getDepth(markerCenters[i][0], markerCenters[i][1])
                depth += d
                print('depth', j, d)
                cv2.waitKey(100)
            depth = depth/5
            M2B_R_tmp, M2B_T_tmp = calM2B(pos[3:], pos[:3], self.c2g, M2C[i], depth)
            M2B_R.append(M2B_R_tmp)
            M2B_T.append(M2B_T_tmp)

        print(M2B_R)
        for i in range(2):
            print('Marker', i)
            print(M2B_T[i])


        # midpoint
        self.robot.set_TMPos(np.array([643.039, -115.206, 625.707, -90.001, 0.001, -135]))
        # up midpoint
        self.robot.set_TMPos(np.array([643.039, -115.206, 625.707, -90.001, 0.001, 135]))
        # down
        self.robot.set_TMPos(np.array([643.039, -115.206, 325.707, -90.001, 0.001, 135]),line=True)
        # pick foup on AMM
        self.robot.set_TMPos(np.array([491.040, -267.207, 325.707, -90.001, 0.001, 135]),line=True)
        # lift
        self.robot.set_TMPos(np.array([491.040, -267.207, 425.707, -90.001, 0.001, 135]),line=True)
        # up midpoint
        self.robot.set_TMPos(np.array([643.039, -115.206, 625.707, -90.001, 0.001, -135]))






        # # back to above desk
        # self.robot.set_TMPos(pos+np.array([0, 0, 150,0, 0, 0]))
        # # place on desk
        # self.robot.set_TMPos(pos, line=True)
        # # back
        # self.robot.set_TMPos(self.framecoordinate, line=True)






        F2B, pos = calM2B_R_2(M2B_T)
        print('F2B', F2B)


        
        


        P2F = np.array([[1,0,0,30],
                        [0,1,0,-358],
                        [0,0,1,-52],
                        [0,0,0,1]])

        P2B = np.dot(F2B,P2F)

        pos = rt2xyzuvw(P2B)      
        print('pos', pos)

        # above
        self.robot.set_TMPos(pos+np.array([0, 0, 150,0, 0, 0]),line=True)
        # place
        self.robot.set_TMPos(pos, line=True)



        P2F = np.array([[1,0,0,31],
                        [0,1,0,-358],
                        [0,0,1,-380],
                        [0,0,0,1]])

        P2B = np.dot(F2B,P2F)
        

        pos = rt2xyzuvw(P2B)      
        print('pos', pos)

        self.framecoordinate=np.copy(pos)

        self.robot.set_TMPos(pos, line=True)
        
        
        

    def moveback(self):
        self.robot.set_TMPos(self.framecoordinate, line=True)


        

        


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = AppWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
    # print('test')

