from PyQt5 import QtWidgets, QtGui, QtCore
from robotControl_UI import Ui_Form
import sys
import os

from PyQt5.QtGui import QPixmap
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


class AppWindow(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.show()

        self.LiDar = l515_qt.l515()
        self.LiDar.daemon = True
        # print(self.LiDar.isDaemon())

        self.ui.textEdit_x.setText(str(-325.6122))
        self.ui.textEdit_y.setText(str(18.64))
        self.ui.textEdit_z.setText(str(219.3423))
        self.ui.textEdit_u.setText(str(-179.1046))
        self.ui.textEdit_v.setText(str(-0.271))
        self.ui.textEdit_w.setText(str(179.115))

        self.robot = None

        self.EIH = EIH_calib.calib()

        self.xPos = -1
        self.yPos = -1
        self.distance = -1
        self.calibEstimatedPos = None
        self.west_intr = None
        self.intr = None
        self.g2c = None

        self.tmp = 90

        self.label_1 = QtWidgets.QLabel(self)
        # self.label_1.show()
        self.label_1.move(0, 0)
        self.label_1.resize(6, 6)
        self.label_1.setStyleSheet('border: 3px solid blue; border-radius: 3px;')

        self.ui.pushButton_Init_robot.clicked.connect(self.Init_Robot)
        self.ui.pushButton_setPos.clicked.connect(self.setRobotPos)
        self.ui.pushButton_Init_Pos.clicked.connect(self.Init_Pos)
        self.ui.pushButton_4Marker_Pos.clicked.connect(self.move_each_marker)
        self.ui.pushButton_camera_on.clicked.connect(self.turn_on_camera)
        self.ui.pushButton_saveImg.clicked.connect(self.save_currImg)
        self.ui.pushButton_close_camera.clicked.connect(self.closeCamera)
        self.ui.pushButton_autoCalib.clicked.connect(self.catch)
        self.ui.pushButton_CalibTest.clicked.connect(self.calib_test)
        self.ui.pushButton_clearPoint.clicked.connect(self.clearPoint)
        self.ui.pushButton_Estimated.clicked.connect(self.go_calibEstimatedPos)
        self.ui.pushButton_gripperOpen.clicked.connect(self.btn_gripper_open)
        self.ui.pushButton_gripperClose.clicked.connect(self.btn_gripper_close)
        self.ui.pushButton_match.clicked.connect(self.btn_matching)
        self.ui.pushButton_getPos.clicked.connect(self.getRobotPos)

        self.ui.label_RGB_viewer.mousePressEvent = self.getmousePos

        print(platform.win32_ver())
        print(platform.platform())
        print(platform.system())
        print(platform.machine())
        print(platform.python_compiler())

        # print(sys.getsizeof(float()))
        # t = 123.212
        # print(sys.getsizeof(t))
        # print(type(t))
        # t = np.float32(t)
        # print(t.dtype)
        # print(sys.getsizeof(t))
        # a = 10
        # print(sys.getsizeof(10))
        # print(a.bit_length())
        

    def Init_Robot(self):
        # self.robot = RobotControl()
        self.robot = RobotControl_func.RobotControl_Func()
        # self.robot.get_currPos()
        # self.robot.Robot_move( -325.6122, 18.64053, 219.3423, -179.1046, 2.317129, -179.8313, 30, 10, False, False, True)
        print('Init completed')


    def setRobotPos(self):
        # print(type(self.ui.textEdit_x.toPlainText()))
        x = float(self.ui.textEdit_x.toPlainText())
        y = float(self.ui.textEdit_y.toPlainText())
        z = float(self.ui.textEdit_z.toPlainText())
        u = float(self.ui.textEdit_u.toPlainText())
        v = float(self.ui.textEdit_v.toPlainText())
        w = float(self.ui.textEdit_w.toPlainText())
        self.robot.Robot_move( x, y, z, u, v, w, 30, 10, True, False, True)

    def Init_Pos(self):
        self.robot.set_initPos()
        print('Init pos')

    def getRobotPos(self):
        x, y, z, u, v, w = self.robot.get_currPos()
        print(x, y, z, u, v, w)

    def btn_gripper_open(self):
        self.robot.gripperOpen()

    def btn_gripper_close(self):
        self.robot.gripperClose()


    def move_each_marker(self):
        # self.robot.markerPos_test()
        # self.robot.SetPos( -325.6122, 18.64053, 219.3423, -179.1046, 2.317129, -179.8313, 30, 10, True, False, True)

        x = float(self.ui.textEdit_x.toPlainText())
        y = float(self.ui.textEdit_y.toPlainText())
        z = float(self.ui.textEdit_z.toPlainText())
        u = float(self.ui.textEdit_u.toPlainText())
        v = float(self.ui.textEdit_v.toPlainText())
        w = float(self.ui.textEdit_w.toPlainText())
        pos = np.array([x, y, z, u, v, w])
        self.robot.move_robotPos(pos)
        # i = 0
        # while(1):
        #     if((i % 2) ==  1):
        #         self.robot.Robot_move( -325.6122, 18.64053, 219.3423, -179.1046, 2.317129, -179.8313, 30, 10, True, False, True)
        #     else:
        #         self.robot.Robot_move( -345.6122, 18.64053, 219.3423, -179.1046, 2.317129, -179.8313, 30, 10, True, False, True)
        #     i += 1
    
    def turn_on_camera(self):
        self.LiDar.flag = True
        self.LiDar.setShowLabel(self.ui.label_Depth_viewer, self.ui.label_RGB_viewer)
        # self.LiDar.recording(self.ui.label_Depth_viewer, self.ui.label_RGB_viewer)
        # self.LiDar.update.connect(self.LiDar.recording(self.ui.label_Depth_viewer, self.ui.label_RGB_viewer))
        self.LiDar.start()

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
    
    def auto_calib(self):
        for p in self.EIH.pos:
            self.robot.Robot_move( p[0], p[1], p[2], p[3], p[4], p[5], 30, 10, False, False, True)
            cv2.waitKey(500)

    def catch(self):
        if self.tmp < len(self.EIH.pos):
            p = self.EIH.pos[self.tmp]
            self.robot.Robot_move( p[0], p[1], p[2], p[3], p[4], p[5], 30, 10, False, False, True)
            # cv2.waitKey(500)
            print(self.tmp)
            # self.save_currImg()
            self.tmp += 1

    def match_func(self):
        # print('match')
        if os.path.isfile("BBOX.bmp") == True:
            print('delete')
            os.remove("BBOX.bmp")
        tmp_color = self.LiDar.color_img
        # tmp_color = cv2.resize()
        cv2.imwrite('BBox/sourceImage.bmp', tmp_color)
        os.startfile("G:/West/py_program/BBox/RST_API.exe")
        img = None
        while(not os.path.isfile("BBOX.bmp") ):
            pass

        img = QPixmap("BBOX.bmp").scaled(864, 486, QtCore.Qt.KeepAspectRatio)
        self.ui.label_match_viewer.setPixmap(img)

        # if os.path.isfile("BBox/BBOX.bmp") == True:
        #     print('delete')
        #     os.remove("BBox/BBOX.bmp")

       
    def btn_matching(self):
        self.match_func()

        
        

    def getmousePos(self, event):
        print(event.x() * 1920 / 864)
        print(event.y() * 1080 / 486)
        self.xPos = event.x() * 1920 / 864
        self.yPos = event.y() * 1080 / 486

        tmp_depth = self.LiDar.depth_img
        self.distance = self.LiDar.depth_frame.as_depth_frame().get_distance(int(self.xPos), int(self.yPos))
        print(self.distance)
        self.label_1.move(int(event.x() + 140 - 5), int(event.y() + 30 - 5))
        self.label_1.show()
        # self.label_1.hide()

       
        
        
    def clearPoint(self):
        self.label_1.hide()

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

        matchingPos = np.loadtxt('BBOX.txt', delimiter=',')
        print(matchingPos)
        for i in range(len(matchingPos)):
            xPos = matchingPos[i][0]
            yPos = matchingPos[i][1]
            zPos = self.LiDar.depth_frame.as_depth_frame().get_distance(int(xPos), int(yPos))

            self.get_L515_intrinsic()

            # LiDar function transform 2D(u, v, d) to 3D(X, Y, Z) 
            # ==========================================================
            pos_rs = rs.rs2_deproject_pixel_to_point( self.intr, [float(xPos), float(yPos)], float(zPos))

            # init position
            x = -325.6122
            y = 18.64
            z = 219.3423
            u = -179.1046
            v = -0.271
            w = 179.115
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

            self.robot.gripperOpen()

            self.robot.Robot_move( pos_plane[0], pos_plane[1] + 10, 133.7, u, v, w, 30, 10, True, False, True)


            self.robot.Robot_move( pos_plane[0], pos_plane[1] + 10, 44.29, u, v, w, 30, 10, True, False, True)
            
            self.robot.gripperClose()

            self.Init_Pos()

            # time.sleep(200)

            targetX = -293.984
            targetY = 357.826
            targetZ = 288.197


            self.robot.Robot_move( targetX - (i % 3) * (80), targetY + int(i / 3) * (80), targetZ, u, v, w, 30, 10, True, False, True)

            self.robot.Robot_move( targetX - (i % 3) * (80), targetY + int(i / 3) * (80), 110.0, u, v, w, 30, 10, True, False, True)
            self.robot.gripperOpen()

            self.robot.Robot_move( targetX - (i % 3) * (80), targetY + int(i / 3) * (80), targetZ, u, v, w, 30, 10, True, False, True)
            self.Init_Pos()

            self.btn_matching()




    





def main():
    app = QtWidgets.QApplication(sys.argv)
    w = AppWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

