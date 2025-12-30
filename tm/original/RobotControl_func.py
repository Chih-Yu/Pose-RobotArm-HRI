# import c++/clr dll
import pythoncom
pythoncom.CoInitialize()
import clr
from clr import System
# test = System.Reflection.Assembly.Load(System.IO.File.ReadAllBytes('G:/West/RobotArm/dll_test/robotDll_test/x64/Debug/robotDll_test.dll'))
# dll_ref = System.Reflection.Assembly.LoadFile(('G:/West/RobotArm/dll_test/robotDll_test/x64/Debug/robotDll_test.dll'))
# print(dll_ref.FullName)
# print(dll_ref.Location)
# clr.AddReference('G:/West/RobotArm/RobotSocket/x64/Debug/RobotSocket')
clr.AddReference('G:/West/RobotArm/dll_test/robotDll_test/x64/Debug/robotDll_test')
clr.AddReference('G:/West/RobotArm/dll_test/robotDll_test/x64/Debug/RCAPINet')
import System.Reflection
from System import String, Char, Int32
# import dll namespace and class
# from myself dll
from CppCLR_WinformsProjekt import Form1
import RobotSocketControl 
# from EPSON dll
import RCAPINet

import ctypes
from ctypes import *

import time
import random
import numpy as np




class RobotControl_Func():
    def __init__(self):
        super().__init__()

        # some_struct_type = dll_ref.GetType('CppCLR_WinformsProjekt.Form1')
        # self.robot = System.Activator.CreateInstance(some_struct_type)
        # self.robot

        s = String.Overloads[Char, Int32]('A', 10)
        print(s)

        # self.m_spel = RCAPINet.Spel() 
        # self.spel_init()
        # print(RCAPINet.Spel.Member)
       

        # self.robot = Form1()
        # print(self.robot.robot)
        # self.robot.robot = RobotSocketControl.Robot()
        print('test')
        # self.robot.robot.SetRobotName(0)
        # print(self.robot)
        # print(self.robot.robot)
        # print(self.robot.robot.GetRobotName())
        # 0 for EPSON
        # if(self.robot.robot.GetRobotName() == 0): 
        #     self.robot.m_spel = RCAPINet.Spel()

        # self.robot.btn_Init_Epson_Click()
        # print(self.robot.robot.GetRobotName())

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.Rx = 0
        self.Ry = 0
        self.Rz = 0
        self.speed = 0
        self.accel = 0

        self.power = True
        self.flip = False
        self.linear = True
        self.tool_id = 1

    def Robot_move(self, x, y, z, Rx, Ry, Rz, speed, accel, power, flip, linear):
        self.xPos = x
        self.yPos = y
        self.zPos = z
        self.Rx = Rx
        self.Ry = Ry
        self.Rz = Rz

        self.robot.SetPos( np.float32(x), np.float32(y), np.float32(z), np.float32(Rx), np.float32(Ry), np.float32(Rz), speed, accel, power, flip, linear)

    def get_currPos(self):
        self.robot.GetCurPos_Python()
        self.xPos = self.robot.xPos
        self.yPos = self.robot.yPos
        self.zPos = self.robot.zPos
        self.Rx = self.robot.uPos
        self.Ry = self.robot.vPos
        self.Rz = self.robot.wPos
        print(self.robot.xPos)
        print(self.robot.yPos)
        print(self.robot.zPos)
        print(self.robot.uPos)
        print(self.robot.vPos)
        print(self.robot.wPos)

        return self.xPos, self.yPos, self.zPos, self.Rx, self.Ry, self.Rz

    def set_initPos(self):
        # self.Robot_move(np.float32(-325.6122), np.float32(18.64), np.float32(219.3423), np.float32(-179.1046), np.float32(0.0), np.float32(179.155), 30, 10, True, False, True)
        self.move_robotInitPos()
    
    def markerPos_test(self):
        self.robot.btn_Func1_Demo_Click()

    def gripperOpen(self):
        self.robot.GripperOpen()

    def gripperClose(self):
        self.robot.GripperClose()

    def move_robotPos(self, pos):
        try:
            if(not self.m_spel.MotorsOn):
                self.m_spel.MotorsOn = True
                self.m_spel.PowerHigh = True
                self.m_spel.Speed(10, 10, 10)
                self.m_spel.Accel(5, 5, 5, 5, 5, 5)
                self.m_spel.Tool(1)


            p = RCAPINet.SpelPoint(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])

            p.Wrist = RCAPINet.SpelWrist.NoFlip
            # if (flip)
            #     p->Wrist = SpelWrist::Flip;
            p.Local = 0 # if there is no Local, put 0

            print('move p')
            self.m_spel.SetPoint(10, p) # PointNumber, SpelPoint; so you can set a point number with 6 poses value

            if (True):
                self.m_spel.Move(np.int32(10))	# A robot will move to a point according to an input point number
            else:
                self.m_spel.Go(10)
        except Exception as e:
            print(e)

        # x, y, z, u, v, w = self.get_currPos()
        self.spel_getPos()

    def move_robotInitPos(self):
        try:
            if(not self.m_spel.MotorsOn):
                self.m_spel.MotorsOn = True
                self.m_spel.PowerHigh = True
                self.m_spel.Speed(10, 10, 10)
                self.m_spel.Accel(5, 5, 5, 5, 5, 5)
                self.m_spel.Tool(1)


            p = RCAPINet.SpelPoint(-325.6122, 18.64, 219.3423, -179.1046, 0.0, 179.155)

            p.Wrist = RCAPINet.SpelWrist.NoFlip
            # if (flip)
            #     p->Wrist = SpelWrist::Flip;
            p.Local = 0 # if there is no Local, put 0
            

            print('move p')
            self.m_spel.SetPoint(10, p) # PointNumber, SpelPoint; so you can set a point number with 6 poses value

            if (True):
                self.m_spel.Move(np.int32(10))	# A robot will move to a point according to an input point number
            else:
                self.m_spel.Go(10)
        except Exception as e:
            print(e)

        # x, y, z, u, v, w = self.get_currPos()
        self.spel_getPos()

    def spel_init(self):
        self.m_spel.Initialize()
        self.m_spel.Project = "c:\\EpsonRC70\\Projects\\Epson_C4601S\\Epson_C4601S.sprj"
        # self.m_spel.EventReceived += RCAPINet.Spel.EventReceivedEventHandler(self, self.m_spel_EventReceived)

        self.m_spel.EnableEvent(RCAPINet.SpelEvents.AllTasksStopped, True)
        self.m_spel.ResetAbortEnabled = False

        self.DisableMsgDispatch = True
        # self.m_spel.AsyncMode = True



        #  Choose a tool
        self.tool_id = 1
        self.m_spel.Tool(self.tool_id)

        #  Flip or not
        robot_flip = False

        a = self.m_spel.GetRobotPos(RCAPINet.SpelRobotPosType.World, 0, self.tool_id, 0)
        for p in a:
            print(p)

    
    def spel_getPos(self):
        a = self.m_spel.GetRobotPos(RCAPINet.SpelRobotPosType.World, 0, self.tool_id, 0)
        pos = []
        for i in range(6):
            pos.append(a[i])

        print(pos)

    # def m_spel_EventReceived(self, sender, e):
    #     pass



    
