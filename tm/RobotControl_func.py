# import pythoncom
# pythoncom.CoInitialize()

import time
import random
import numpy as np
from PyQt5.QtCore import QMutex, QObject, pyqtSlot, QThreadPool, QRunnable, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication


import rclpy
from rclpy.node import Node
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../../', 'tm_msgs/msg'))
# print(sys.path)
# from tm_msgs import msg
from tm_msgs.msg import *
from tm_msgs.srv import *
import cv2


mutex = QMutex()


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

class setPositionClientAsync(Node):

    def __init__(self, pos, line):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetPositions, 'set_positions')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetPositions.Request()
        self.pos = pos
        self.line = line

    def send_request(self):
        self.req.motion_type = self.line
        self.req.positions = self.pos
        self.req.velocity = 1.0
        self.req.acc_time = 0.2
        self.req.blend_percentage = 0
        self.req.fine_goal = False
        print(self.req)
        self.future = self.cli.call_async(self.req)

class askstaClientAsync(Node):

    def __init__(self):
        super().__init__('ask_sta__client_async')
        self.cli = self.create_client(AskSta, 'ask_sta')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AskSta.Request()

    def send_request(self):
        self.req.subcmd = '01'
        self.req.subdata = str(1)
        self.req.wait_time = 1.0
        print(self.req)
        self.future = self.cli.call_async(self.req)

class seteventClientAsync(Node):

    def __init__(self):
        super().__init__('set_event__client_async')
        self.cli = self.create_client(SetEvent, 'set_event')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetEvent.Request()

    def send_request(self):
        self.req.func = SetEvent.Request.TAG
        self.req.arg0 = 1
        self.req.arg1 = 0
        print(self.req)
        self.future = self.cli.call_async(self.req)


# Robot Arm move
class myMitter(QObject):
    done = pyqtSignal(bool)

class worker(QRunnable):
        def __init__(self, pos, speed, line):
            super(worker, self).__init__()

            self.pos = pos
            self.speed = speed
            self.line = line
            self.mitter = myMitter()

        @pyqtSlot()
        def run(self):
            try:
                mutex.lock()

                if self.line == False:
                    minimal_client = setPositionClientAsync(self.pos, SetPositions.Request.PTP_T)
                    minimal_client.send_request()

                    while rclpy.ok():
                        print('here')
                        rclpy.spin_once(minimal_client)
                        if minimal_client.future.done():
                            try:
                                response = minimal_client.future.result()
                            except Exception as e:
                                minimal_client.get_logger().info(
                                    'Service call failed %r' % (e,))
                            else:
                                print('ptp')
                            break
                    minimal_client.destroy_node()
                else:
                    minimal_client = setPositionClientAsync(self.pos, SetPositions.Request.LINE_T)
                    minimal_client.send_request()

                    while rclpy.ok():
                        print('here')
                        rclpy.spin_once(minimal_client)
                        if minimal_client.future.done():
                            try:
                                response = minimal_client.future.result()
                            except Exception as e:
                                minimal_client.get_logger().info(
                                    'Service call failed %r' % (e,))
                            else:
                                print('line')
                            break
                    minimal_client.destroy_node()

                minimal_client = seteventClientAsync()
                minimal_client.send_request()

                while rclpy.ok():
                    print('here')
                    rclpy.spin_once(minimal_client)
                    if minimal_client.future.done():
                        try:
                            response = minimal_client.future.result()
                        except Exception as e:
                            minimal_client.get_logger().info(
                                'Service call failed %r' % (e,))
                        else:
                            print('line')
                        break
                minimal_client.destroy_node()
                
                while True:
                    cv2.waitKey(5)
                    minimal_client = askstaClientAsync()
                    minimal_client.send_request()

                    while rclpy.ok():
                        print('here')
                        rclpy.spin_once(minimal_client)
                        if minimal_client.future.done():
                            try:
                                response = minimal_client.future.result()
                            except Exception as e:
                                minimal_client.get_logger().info(
                                    'Service call failed %r' % (e,))
                            else:
                                print('line')
                            break
                        
                    if response.subcmd == '01':
                        data = response.subdata.split(',')
                        if data[1] == 'true':
                            minimal_client.get_logger().info('point %d (Tag %s) is reached', 1, data[0])
                            break
                minimal_client.destroy_node()
                

                # rospy.wait_for_service('tm_driver/ask_sta')
                # rospy.wait_for_service('tm_driver/set_event')
                # rospy.wait_for_service('tm_driver/set_positions')
                # ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)
                # set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
                # set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
                # print(self.pos)


                # if self.line == False:
                #     set_positions(SetPositionsRequest.PTP_T, self.pos, self.speed, 0.4, 0, False)
                # else:
                #     set_positions(SetPositionsRequest.LINE_T, self.pos, 0.8, 0.1, 0, False)

                # set_event(SetEventRequest.TAG, 1, 0)

                # while True:
                #     rospy.sleep(0.2)
                #     res = ask_sta('01', str(1), 1)
                #     if res.subcmd == '01':
                #         data = res.subdata.split(',')
                #         if data[1] == 'true':
                #             rospy.loginfo('point %d (Tag %s) is reached', 1, data[0])
                #             break
                # minimal_client.destroy_node()
              
            except Exception as e: 
                print(e)


            # self.emitter.done.emit(False)
            self.mitter.done.emit(True)
            # print('emit')
            

            mutex.unlock()

class RobotControl_Func():
    def __init__(self):
        super().__init__()

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.Rx = 0
        self.Ry = 0
        self.Rz = 0
        self.speed = 0
        self.accel = 0

        self.pool = QThreadPool.globalInstance()
        self.pool.setMaxThreadCount(1)

        self.threadDone = True




    def on_worker_done(self, threadDone):
        print(threadDone)
        self.threadDone = threadDone

    def set_TMPos(self, pos, speed = 4, line = False):
        # transself.set_TMPos_new(pos)form to TM robot coordinate
        tmp = []

        tmp.append(pos[0] / 1000)
        tmp.append(pos[1] / 1000)
        tmp.append(pos[2] / 1000)
        tmp.append(pos[3] * np.pi / 180)
        tmp.append(pos[4] * np.pi / 180)
        tmp.append(pos[5] * np.pi / 180)

        # minimal_client = setPositionClientAsync(tmp)
        # minimal_client.send_request()

        # while rclpy.ok():
        #     print('here')
        #     rclpy.spin_once(minimal_client)
        #     print('here')
        #     if minimal_client.future.done():
        #         try:
        #             response = minimal_client.future.result()
        #             print(response)
        #         except Exception as e:
        #             minimal_client.get_logger().info(
        #                 'Service call failed %r' % (e,))
        #         else:
        #             print('suwei')
        #         break
        

        self.threadDone = False
        runnable = worker(tmp, speed, line)
        runnable.mitter.done.connect(self.on_worker_done)
        self.pool.start(runnable)


        count = 0
        #while(self.threadDone == False):
            # magic functon -> 用來更新UI介面
            #QApplication.processEvents()

            
            # if((count % 10000) == 0):
            #     print(self.get_TMPos())
            # count += 1

            # print(self.threadDone)
            # time.sleep(1)
        print('Move')
        

    def get_TMPos(self):
        # listen to 'feedback_states' topic
        minimal_subscriber = getfeedbacklSubscriber()
        rclpy.spin_once(minimal_subscriber)
        data = minimal_subscriber.pos
        current_pos = list(data.tool_pose)
        current_pos[0] = current_pos[0] * 1000
        current_pos[1] = current_pos[1] * 1000
        current_pos[2] = current_pos[2] * 1000
        current_pos[3] = current_pos[3] * 180 / np.pi
        current_pos[4] = current_pos[4] * 180 / np.pi
        current_pos[5] = current_pos[5] * 180 / np.pi
        # print(self.robot)
        minimal_subscriber.destroy_node()

        
        return current_pos
