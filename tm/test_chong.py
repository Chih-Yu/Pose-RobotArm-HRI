import RobotControl_func
import rclpy
from rclpy.node import Node
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../../', 'tm_msgs/msg'))
# print(sys.path)
# from tm_msgs import msg
from tm_msgs.msg import *
from tm_msgs.srv import *
import keyboard

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

robot = RobotControl_func.RobotControl_Func()
rclpy.init(args=None)
minimal_subscriber = getfeedbacklSubscriber()
while True:
    i = input()
    print(i)
    if i == 'w':  # if key 'q' is pressed 
        [x,y,z,u,v,w] = robot.get_TMPos()
        robot.set_TMPos([x+50,y,z,u,v,w])
        print('You Pressed A Key!')  # finishing the loop
    if i == 's':  # if key 'q' is pressed 
        [x,y,z,u,v,w] = robot.get_TMPos()
        robot.set_TMPos([x-50,y,z,u,v,w])
        print('You Pressed A Key!')  # finishing the loop
    if i == 'a':  # if key 'q' is pressed 
        [x,y,z,u,v,w] = robot.get_TMPos()
        robot.set_TMPos([x,y+50,z,u,v,w])
        print('You Pressed A Key!')  # finishing the loop
    if i == 'd':  # if key 'q' is pressed 
        [x,y,z,u,v,w] = robot.get_TMPos()
        robot.set_TMPos([x,y-50,z,u,v,w])
        print('You Pressed A Key!')  # finishing the loop
    else:
        continue
while True:
    try:  # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('w'):  # if key 'q' is pressed 
            [x,y,z,u,v,w] = robot.get_TMPos()
            robot.set_TMPos([x+50,y,z,u,v,w])
            print('You Pressed A Key!')  # finishing the loop
        if keyboard.is_pressed('s'):  # if key 'q' is pressed 
            [x,y,z,u,v,w] = robot.get_TMPos()
            robot.set_TMPos([x-50,y,z,u,v,w])
            print('You Pressed A Key!')  # finishing the loop
        if keyboard.is_pressed('a'):  # if key 'q' is pressed 
            [x,y,z,u,v,w] = robot.get_TMPos()
            robot.set_TMPos([x,y+50,z,u,v,w])
            print('You Pressed A Key!')  # finishing the loop
        if keyboard.is_pressed('d'):  # if key 'q' is pressed 
            [x,y,z,u,v,w] = robot.get_TMPos()
            robot.set_TMPos([x,y-50,z,u,v,w])
            print('You Pressed A Key!')  # finishing the loop
    except:
        continue