#!/usr/bin/env python

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QThread, pyqtSignal

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
# import RobotControl_func

import warnings
warnings.simplefilter("ignore", UserWarning)
sys.coinit_flags = 2
import l515_qt
import pyrealsense2 as rs

##import EIH_calib
#import Epson_EIH

import platform

import rospy
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../../', 'tm_msgs/msg'))
# print(sys.path)
# from tm_msgs import msg
from tm_msgs.msg import *
from tm_msgs.srv import *


class AppWindow(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.show()

        self.LiDar = l515_qt.l515()
        self.LiDar.daemon = True
        # print(self.LiDar.isDaemon())

        self.ui.textEdit_x.setText(str(376.0587))
        self.ui.textEdit_y.setText(str(-436.6104))
        self.ui.textEdit_z.setText(str(548.17))
        self.ui.textEdit_u.setText(str(179.98))
        self.ui.textEdit_v.setText(str(-0.04045901))
        self.ui.textEdit_w.setText(str(-135.05))

        self.robot = None

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
        m = fn.mat()
        self.mtx = m
        self.LiDar.setDist(self.dist, self.mtx)

        self.tmp = 90

        self.label_1 = QtWidgets.QLabel(self)
        # self.label_1.show()
        self.label_1.move(0, 0)
        self.label_1.resize(6, 6)
        self.label_1.setStyleSheet('border: 3px solid blue; border-radius: 3px;')

        self.ui.pushButton_camera_on.clicked.connect(self.turn_on_camera)
        self.ui.pushButton_saveImg.clicked.connect(self.save_currImg)
        self.ui.pushButton_close_camera.clicked.connect(self.closeCamera)
        self.ui.pushButton_autoCalib.clicked.connect(self.auto_calib)
        self.ui.pushButton_CalibTest.clicked.connect(self.calib_test)
        self.ui.pushButton_clearPoint.clicked.connect(self.clearPoint)
        self.ui.pushButton_Estimated.clicked.connect(self.go_calibEstimatedPos)
        self.ui.pushButton_getPos.clicked.connect(self.get_TMPos)
        self.ui.pushButton_setPos.clicked.connect(self.set_TMPos_by_ui)
        self.ui.pushButton_Init_Pos.clicked.connect(self.set_initPos)


        self.ui.label_RGB_viewer.mousePressEvent = self.getmousePos

        rospy.init_node("RobotControl")

        self.ask_sta = None
        self.t = None
        self.count = 0

        

        

    def get_TMPos(self):
        # listen to 'feedback_states' topic
        data = rospy.wait_for_message("/feedback_states", FeedbackState, timeout=None)
        # print(data.tool_pose)
        print(data.tcp_speed)
        self.robot = list(data.tool_pose)
        self.robot[0] = self.robot[0] * 1000
        self.robot[1] = self.robot[1] * 1000
        self.robot[2] = self.robot[2] * 1000
        self.robot[3] = self.robot[3] * 180 / np.pi
        self.robot[4] = self.robot[4] * 180 / np.pi
        self.robot[5] = self.robot[5] * 180 / np.pi
        # print(self.robot)

        self.ui.textEdit_x.setText(str(self.robot[0]))
        self.ui.textEdit_y.setText(str(self.robot[1]))
        self.ui.textEdit_z.setText(str(self.robot[2]))
        self.ui.textEdit_u.setText(str(self.robot[3]))
        self.ui.textEdit_v.setText(str(self.robot[4]))
        self.ui.textEdit_w.setText(str(self.robot[5]))

    def set_TMPos(self, pos):
        # using 'tm_driver/ask_item' service
        rospy.wait_for_service('tm_driver/set_positions')
        set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)

        tmp = []

        tmp.append(pos[0] / 1000)
        tmp.append(pos[1] / 1000)
        tmp.append(pos[2] / 1000)
        tmp.append(pos[3] * np.pi / 180)
        tmp.append(pos[4] * np.pi / 180)
        tmp.append(pos[5] * np.pi / 180)

        res1 = set_positions(SetPositionsRequest.PTP_T, tmp, 0.8, 0.4, 0, False)
        # print(res1)

    
    def set_TMPos_by_ui(self):
        x = float(self.ui.textEdit_x.toPlainText())
        y = float(self.ui.textEdit_y.toPlainText())
        z = float(self.ui.textEdit_z.toPlainText())
        u = float(self.ui.textEdit_u.toPlainText())
        v = float(self.ui.textEdit_v.toPlainText())
        w = float(self.ui.textEdit_w.toPlainText())

        self.set_TMPos([x, y, z, u, v, w])


    def set_initPos(self):
        x = 376.0587
        y = -436.6104
        z = 548.17
        u = 179.98
        v = -0.04045901
        w = -135.05

        self.set_TMPos([x, y, z, u, v, w])


    
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
        self.set_TMPos(self.EIH.pos[self.count])
        self.count += 1

            
        # # using services
        # rospy.wait_for_service('tm_driver/ask_sta')
        # rospy.wait_for_service('tm_driver/set_event')
        # self.ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)
        # set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)


        # for idx in range(4):
        #     # print(p)
        #     self.set_TMPos(self.EIH.pos[idx])
        #     set_event(SetEventRequest.TAG, idx + 1, 0)
        #     # cv2.waitKey(100)
        #     # print(sta_response)
        #     # self.save_currImg()

        # self.t = TM_thread(self.set_TMPos, self.set_initPos, self.save_currImg, self.EIH, self.ask_sta)
        # self.t.daemon = True
        # print('r')
        # self.t.start()
        # print('r')

        
        # for i in range(0, len(self.EIH.pos), 4):
        #     print(i)
        #     for idx in range(4):
        #         # print(p)
        #         self.set_TMPos(self.EIH.pos[i + idx])
        #         set_event(SetEventRequest.TAG, idx + 1, 0)
        #         cv2.waitKey(100)
        #         # print(sta_response)
        #         # self.save_currImg()

        #     self.t = TM_thread(self.set_TMPos, self.set_initPos, self.save_currImg, self.EIH, self.ask_sta)
        #     self.t.daemon = True
        #     print('r')
        #     self.t.start()
        #     print('r')

        # rest = len(self.EIH.pos) % 4
        # for idx in range(len(self.EIH.pos)-rest, len(self.EIH.pos)):
        # #     # print(p)
        #     self.set_TMPos(self.EIH.pos[idx])
        #     set_event(SetEventRequest.TAG, idx + 1, 0)
        #     cv2.waitKey(100)
        #     # print(sta_response)
        #     # self.save_currImg()

        # self.t = TM_thread(self.set_TMPos, self.set_initPos, self.save_currImg, self.EIH, self.ask_sta)
        # self.t.daemon = True
        # print('r')
        # self.t.start()
        # print('r')


        # # ask sta to check QueueTag state
        # i = 0
        # while i < len(self.EIH.pos):
        #     rospy.sleep(0.2)
        #     res = ask_sta('01', str(i + 1), 1)
        #     print(res.subcmd)
        #     if res.subcmd == '01':
        #         data = res.subdata.split(',')
        #         if data[1] == 'true':
        #             rospy.loginfo('point %d (Tag %s) is reached', i + 1, data[0])
        #             self.save_currImg()
        #             i = i + 1

        # self.set_initPos()

    def sta_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ': %s', msg.subdata)
        if msg.subcmd == '01':
            data = msg.subdata.split(',')
            if data[1] == 'true':
                rospy.loginfo('point (Tag %s) is reached', data[0])

    # def catch(self):
    #     if self.tmp < len(self.EIH.pos):
    #         p = self.EIH.pos[self.tmp]
    #         self.robot.Robot_move( p[0], p[1], p[2], p[3], p[4], p[5], 30, 10, False, False, True)
    #         # cv2.waitKey(500)
    #         print(self.tmp)
    #         # self.save_currImg()
    #         self.tmp += 1

        

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
        z = 410.5
        u = 179.98
        v = -0.04045901
        w = -135.05

        self.set_TMPos([x, y, z, u, v, w])





class TM_thread(QThread):
    def __init__(self, set_pos, set_initPos, saveImg, calibPos, ask_sta):
        super(TM_thread,self).__init__()
        self.set_pos_func = set_pos
        self.set_initPos_func = set_initPos
        self.saveImg_func = saveImg
        self.EIH = calibPos
        self.ask_sta = ask_sta
        

    def run(self):
        # ask sta to check QueueTag state
        i = 0
        while i < 4:
            rospy.sleep(0.2)
            res = self.ask_sta('01', str(i + 1), 1)
            # print(res.subdata)
            if res.subcmd == '01':
                data = res.subdata.split(',')
                if data[1] == 'true':
                    rospy.loginfo('point %d (Tag %s) is reached', i + 1, data[0])
                    self.saveImg_func()
                    i = i + 1

        # self.set_initPos_func()





def main():
    app = QtWidgets.QApplication(sys.argv)
    w = AppWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

