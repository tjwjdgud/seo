#!/usr/bin/env python

# encoding: utf-8

import getpass
import threading

from mono_common import *

from sensor_msgs.msg import CompressedImage

from dynamic_reconfigure.server import Server

from dynamic_reconfigure.client import Client

from transbot_mono.cfg import monoTrackerPIDConfig

from geometry_msgs.msg import Twist

 

image_width = 640

image_height = 480

class mono_Tracker:

    def __init__(self):

        rospy.init_node("mono_Tracker", anonymous=False)

        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        self.condition = threading.Condition()

        rospy.on_shutdown(self.cancel)

        self.target_servox = 90

        self.target_servoy = 90

        self.point_pose = (0, 0, 0)

        self.circle = (0, 0, 0)

        self.hsv_range = ()

        self.dyn_update = True

        self.select_flags = False

        self.gTracker_state = False

        self.windows_name = 'frame'

        self.cols, self.rows = 0, 0

        self.Mouse_XY = (0, 0)

        self.index = 2

        self.color = color_follow()

        self.ros_ctrl = ROSCtrl()

        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT' ,"color"]

        self.img_flip = rospy.get_param("~img_flip", False)

        self.tracker_type = rospy.get_param("~tracker_type", 'KCF')

        self.VideoSwitch = rospy.get_param("~VideoSwitch", True)

        self.user_name = getpass.getuser()

        text_path = '/home/'+self.user_name+'/transbot_ws/src/transbot_mono/scripts'

        self.hsv_text = text_path+"/MonoTrackerHSV.text"

        Server(monoTrackerPIDConfig, self.dynamic_reconfigure_callback)

        self.dyn_client = Client("/mono_Tracker", timeout=60)

        self.mono_PID = (20, 0, 2)

        self.scale = 1000

        self.PID_init()

        print("OpenCV Version: ",cv.__version__)

        self.tracker_type = self.tracker_types[self.index]

        self.Track_state = 'init'

        if self.VideoSwitch == False:

            from cv_bridge import CvBridge

            self.bridge = CvBridge()

            self.img_topic = rospy.get_param("~camera", "/usb_cam/image_raw/compressed")

    def cancel(self):

        self.Reset()

        self.ros_ctrl.cancel()

        if self.VideoSwitch==False: self.__sub_img.unregister()

        cv.destroyAllWindows()

        

    def update(self, x, y, z, th, speed, turn):

        self.condition.acquire()

        self.x = x

        self.y = y

        self.z = z

        self.th = th

        self.speed = speed

        self.turn = turn

        

        twist = Twist()

        twist.linear.x = self.x * self.speed

        twist.linear.y = self.y * self.speed

        twist.linear.z = self.z * self.speed

        twist.angular.x = 0

        twist.angular.y = 0

        twist.angular.z = self.th * self.turn

        

        self.publisher.publish(twist)

        # Notify publish thread that we have a new message.

        self.condition.notify()

        self.condition.release()

        

            

    def Reset(self):

        self.ros_ctrl.PWM_Reset()

        self.hsv_range = ()

        self.circle = (0, 0, 0)

        self.Mouse_XY = (0, 0)

        self.Track_state = 'init'

        self.target_servox = 90

        self.target_servoy = 90

        rospy.loginfo("PWM init succes!!!")

    def execute(self, point_x, point_y):

        # rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))

        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])

        if self.img_flip == True:

            self.target_servox -= x_Pid

            self.target_servoy -= y_Pid

        else:

            self.target_servox += x_Pid

            self.target_servoy -= y_Pid

        if self.target_servox >= 180:

            self.target_servox = 180

        elif self.target_servox <= 0:

            self.target_servox = 0

        if self.target_servoy >= 180:

            self.target_servoy = 180

        elif self.target_servoy <= 0:

            self.target_servoy = 0

        # rospy.loginfo("target_servox: {}, target_servoy: {}".format(self.target_servox, self.target_servoy))

        self.ros_ctrl.PWMServo_topic(1, self.target_servox)

        self.ros_ctrl.PWMServo_topic(2, self.target_servoy)

    def dynamic_reconfigure_callback(self, config, level):

        self.scale = config['scale']

        self.mono_PID = (config['Kp'], config['Ki'], config['Kd'])

        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),

                          (config['Hmax'], config['Smax'], config['Vmax']))

        self.PID_init()

        return config

    def PID_init(self):

        self.PID_controller = simplePID(

            [0, 0],

            [self.mono_PID[0] / float(self.scale), self.mono_PID[0] / float(self.scale)],

            [self.mono_PID[1] / float(self.scale), self.mono_PID[1] / float(self.scale)],

            [self.mono_PID[2] / float(self.scale), self.mono_PID[2] / float(self.scale)])

    

    def process(self, A_center_x, A_height, A_width):

        # param action: [113 or 'q':退出]，[114 or 'r':重置]，[105 or 'i'：识别]，[32：开始追踪]

        center_x = A_center_x

        width = A_width

        high = A_height

        threading.Thread(target=self.control, args=(center_x, width, high)).start()

        

    def control(self, A_center_x,A_height,A_width):

        #pub_thread = PublishThread(repeat)

        x = 1

        y = 0

        z = 0

        th = 1

        status = 0

        speed = 0.2

        turn = 0.5

        output = ['recog', 'back and forth', 'left and right', 0.2, 0.5, 320, 320]

        size_y = A_height

        size_x = A_width

        center_x = A_center_x

        while(1):              

                '''

                if(size_y < 200):

                    speed = 0

                    turn = 0

                    output[0] = "No"                    

                

                else:

                    if(center_x < (image_width/2*0.95)):

                        th = 1

                        turn = ((image_width/2) - center_x)/image_width*3.0

                        output[2] = "Left"      

                    elif(center_x > (image_width - image_width/2*0.95)):

                        th = -1

                        turn = (center_x - (image_width/2))/image_width*3.0

                        output[2] = "Right"

                    else:

                        turn = 0

                        output[2] = "Straight"

                    

                    if(size_x < image_width/2):#320qq

                        speed = ((image_width/2) - size_x) / (image_width/2) * 2.0

                        output[1] = "Forward"

                        x = 1

                    

                    elif(size_x>image_width/2* 1.05): #336

                        output[1] = "Stop"

                        x = 0

                        speed = 0                        

                        #output[1] = "Back"

                        #speed = 0.2

                        #x = -1

                        #th = th * -1

                    else:

                        output[1] = "Stop"

                        x = 0

                        speed = 0

                    

                    if(0 < speed < 0.2):

                        speed = 0.2

                    if(speed > 1.5):

                        speed  = 1.5

                                     '''   

                #print(turn, speed)

                output[3] = speed

                output[4] = turn

                output[5] = center_x

                output[6] = size_x

                #print("\033[2J", end='')

                #print(f"{output[0]} {output[1]} {output[2]} \nSpeed: {output[3]:.2f} Angular: {output[4]:.2f} \nCentor: {output[5]:.2f} / with: {output[6]:.2f}")

                #print(f"Height: {self.size_y:.2f}")

                self.update(x, y, z, th, speed, turn)    

    

if __name__ == '__main__':

    mono_Tracker = mono_Tracker()

    print(mono_Tracker.VideoSwitch)

    center_x = 0

    center_y = 0

    height = 0

    width = 0

    if mono_Tracker.VideoSwitch==False: rospy.spin()

   

    else:

        capture = cv.VideoCapture(0)

        

        #gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY) 

        cv_edition = cv.__version__

        if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))

        else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)

        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

        print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))

        while capture.isOpened():

            

            start = time.time()

            ret, frame = capture.read()

            action = cv.waitKey(10) & 0xFF

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #make grayscale

            lower_body_cascade = cv.CascadeClassifier('/home/jetson/Downloads/haarcascade_fullbody.xml')

            lower_body = lower_body_cascade.detectMultiScale(gray, 3, minNeighbors=1,minSize=(120, 250))

 

            for (x,y,w,h) in lower_body:

                cv.rectangle(gray,(x,y),(x+w,y+h),(0,0,255),3)

                center_x = int(x+w/2)

                center_y = int(y+h/2)

                print("center_x = " + str(int(x+w/2)) + ", center_y = " + str(int(y+h/2)) + ", width = " + str(w) + ", height = " + str(h))

            end = time.time()

            mono_Tracker.process(center_x, height, width)

            fps = 1 / (end - start)

            text = "FPS : " + str(int(fps))

            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

            cv.imshow('frame', gray)

            if action == ord('q') or action == 113: break

        capture.release()

        cv.destroyAllWindows()
