import rospy

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# OpenCV2 for saving an image
import numpy as np
import cv2 as cv

# get range from qr/target/elp
from math import tan, radians
import random as rng

# to move robot
from geometry_msgs.msg import Twist

# Instantiate CvBridge for converting ROS Image messages to OpenCV2
bridge = CvBridge()
# Instantiate VERBOSE variable globally for verbose mode
VERBOSE = False


class LineSensor:

    def __init__(self):
        self.sensor_count = 8 # -1
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        self.key = False
        self.flag_left = False
        self.counter = [0 for i in range(self.sensor_count)]
        width = 640
        height = 360

        self.img = np.zeros((height,width),np.uint8)
        print(self.img.shape)
        self.sense_pos_x = np.array([width*i//self.sensor_count for i in range(1,self.sensor_count)]).astype(int)
        self.sense_pos_y = 50
        # initialize node

        # subscribe to sensors/camera to get image
        self.image_topic = "/camera/image_raw"

        self.timestamp = rospy.Time.now()

        # Subscribe to Img
        self.img_sub = rospy.Subscriber(
                self.image_topic, Image, self.callback, callback_args="img"
        )
        
        
    def callback(self, msg, args):
        """
        This function is called when the image_topic is published.
        It gets the image from the topic and convert from ROS Image msgs to OpenCV2 Image.
        """
        print("get img")
        try:
            # Convert your ROS Image message to OpenCV2
            if args == "img":
                img = bridge.imgmsg_to_cv2(msg, "bgr8")
                img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                self.img = cv.resize(img, (640, 360))
                self.key = True
        except CvBridgeError as e:
            print(Warning("Conversion failed: {}".format(e)))
        
    def timer_callback(self, msg):
        if not self.key:
            print("no img")
            return
        # print("msg", msg)
        print("timer callback")
        cv.imwrite(f'out/{self.err}_{self.counter[self.err+3]}.jpeg', self.img)
        self.counter[self.err+3] += 1
        self.t_last = rospy.Time.now().nsecs

    def get_position(self):
        """
        
        """
        img = self.img
        print(img.shape)
        cropped = img[150:250,0:640]
        ret,img = cv.threshold(cropped,127,255,cv.THRESH_BINARY)
        tl, tr, bl, br = (),(),(),()
        # Cropping an image
        w = img.shape[1]
        print("shape ",img.shape)
        sense_pos_x = self.sense_pos_x
        sense_pos_y = self.sense_pos_y
        print(len(sense_pos_x))
        print(sense_pos_x)
        copy = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        for i in sense_pos_x:
            cv.line(copy,(i,sense_pos_y-10),(i,sense_pos_y+10),(0,0,255),3)
        cv.imshow("ril", self.img)
        cv.imshow("test", copy)
        cv.waitKey(1)
        sensed = []
        for i in range(len(sense_pos_x)):
            sensed.append(img[sense_pos_y, sense_pos_x[i]])
        return sensed    
        
    
class Robot(LineSensor):
    def __init__(self):
        super().__init__()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    def get_camera_image(self):
        return self.img
    

    def motor(self, speed, rotation_speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = rotation_speed
        self.cmd_vel_pub.publish(twist)

    def stop_motor(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def demo_move(self):
        speed = 0.15
        twist = Twist()
        sensed = self.get_position()
        self.err = 0
        if(sensed == None):
            print("no Img")
            return 
        print(sensed)
        num = len(sensed)
        if(sensed[num//2] == 0):
            twist.linear.x = speed
            twist.angular.z = 0.0
            self.err = 0
        elif(sensed[num//2 - 1] == 0 or sensed[num//2 + 1] == 0):
            if(sensed[num//2-1] ==0):
                twist.linear.x = speed
                twist.angular.z = 0.1
                self.err = -1
            else:
                twist.linear.x = speed
                twist.angular.z = -0.1
                self.err = 1
        elif(sensed[num//2 - 2] == 0 or sensed[num//2 + 2] == 0):
            if(sensed[num//2 - 2] ==0):
                twist.linear.x = speed
                twist.angular.z = 0.2
                self.err = -2
            else:
                twist.linear.x = speed
                twist.angular.z = -0.2
                self.err = 2
        elif(sensed[num//2 - 3] == 0 or sensed[num//2 + 3] == 0):
            if(sensed[num//2 - 3] ==0):
                twist.linear.x = speed
                twist.angular.z = 0.5
                self.err = 3
            else:
                twist.linear.x = speed
                twist.angular.z = -0.5
                self.err = -3
        else:
            if self.flag_left:
                twist.linear.x = speed - 0.05
                twist.angular.z = 0.3
            else:
                twist.linear.x = speed -0.05
                twist.angular.z = -0.3
        if self.key:
            self.cmd_vel_pub.publish(twist)
            self.key = False
        # else:
        #     twist.linear.x = 0.01
        #     twist.angular.z = 0
        #     self.cmd_vel_pub.publish(twist)

    def capture_data(self):
        cv.imwrite(f"out/test{self.counter}.jpg", self.img)
        self.counter += 1
        
        

if __name__ == "__main__":
    rospy.init_node("line_follow")
    try:
        robot = Robot()
        last = rospy.Time.now()
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            
            if rospy.Time.now() - last > rospy.Duration(5):
                rospy.loginfo("[Robot] Heartbeat every 5 seconds")
                last = rospy.Time.now()
            robot.demo_move()
            r.sleep()

    except rospy.ROSInterruptException:
        pass