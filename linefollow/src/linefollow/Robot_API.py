import rospy

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# OpenCV2 for saving an image
import numpy as np
import cv2 as cv

# get range from qr/target/elp
from math import tan, radians, sqrt
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
        self.key = False
        self.flag_left = False
        self.counter = [0 for i in range(self.sensor_count)]
        width = 640
        height = 360

        self.img = np.zeros((height,width),np.uint8)
        # print(self.img.shape)
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
        # print("get img")
        try:
            # Convert your ROS Image message to OpenCV2
            if args == "img":
                img = bridge.imgmsg_to_cv2(msg, "bgr8")
                img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                self.img = cv.resize(img, (640, 360))
                self.key = True
                
        except CvBridgeError as e:
            rospy.logwarn(Warning("Conversion failed: {}".format(e)))

    def get_position(self):
        """
        
        """
        img = self.img
        # print(img.shape)
        cropped = img[150:250,0:640]
        ret,img = cv.threshold(cropped,127,255,cv.THRESH_BINARY)
        tl, tr, bl, br = (),(),(),()
        # Cropping an image
        w = img.shape[1]
        # print("shape ",img.shape)
        sense_pos_x = self.sense_pos_x
        sense_pos_y = self.sense_pos_y
        # print(len(sense_pos_x))
        # print(sense_pos_x)
        copy = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        for i in sense_pos_x:
            cv.line(copy,(i,sense_pos_y-10),(i,sense_pos_y+10),(0,0,255),3)
        cv.imshow("ril", self.img)
        cv.imshow("sensor", copy)
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
    
    def get_lines(self, line_black = True, threshold = 127):
        """
        Get lines from image
        args:
            line_black: True for black, False for white
            threshold: threshold of color
        return:
            list of lines 1 for line, 0 for not line
        """
        sensed = []
        for i in self.get_position():
            if line_black:
                if i < threshold:
                    sensed.append(1)
                else:
                    sensed.append(0)
            else:
                if i > threshold:
                    sensed.append(1)
                else:
                    sensed.append(0)

        return sensed

    def move(self, speed : float, rotation_speed : float):
        """
        Move the robot with linear velocity and steer robot with angular velocity
        args:
            speed: 
            rotation_speed: 
        """
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = rotation_speed
        self.cmd_vel_pub.publish(twist)

    def motor(self, speedl:int, speedr:int, speed_const = 0.2, omega_const = 0.3):
        """
        Move robot with difference speed between left and right wheel
        args:
            speedl: (-100 - 100)% speed of left wheel
            speedr: (-100 - 100)% speed of right wheel
        """
        twist = Twist()        
        steer =  (speedr - speedl)/100
        twist.linear.x = ((speedl/100) + (speedr/100))/2 * speed_const
        twist.angular.z = steer*omega_const
        # rospy.loginfo(f"speedl: {speedl}, speedr: {speedr}")
        # rospy.loginfo(f"v: {twist.linear.x}, omega: {twist.angular.z}")
        self.cmd_vel_pub.publish(twist)
        

    def stop(self):
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
            rospy.logwarn("no Img")
            return 
        # print(sensed)
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

    def check_color(self):
        pass

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