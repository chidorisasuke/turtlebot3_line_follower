
import rospy

# ROS Image message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np
import csv


# get range from qr/target/elp
from math import tan, radians
import random as rng

# to move robot
from geometry_msgs.msg import Twist


class Fusion:
    def __init__(self) :
        self.fields_imu = ['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z']
        self.fields_odom = ['timestamp', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.callback_imu)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.data_imu = []
        self.data_odom = []
        
    def callback_imu(self, data: Imu):
        lis = []
        lis.append(data.header.stamp.secs)
        lis.append(data.orientation.x)
        lis.append(data.orientation.y)
        lis.append(data.orientation.z)
        lis.append(data.orientation.w)
        lis.append(data.angular_velocity.x)
        lis.append(data.angular_velocity.y)
        lis.append(data.angular_velocity.z)
        lis.append(data.linear_acceleration.x)
        lis.append(data.linear_acceleration.y)
        lis.append(data.linear_acceleration.z)
        self.data_imu.append(lis)
        

    def callback_odom(self, data: Odometry):
        lis = []
        lis.append(data.header.stamp.secs)
        lis.append(data.pose.pose.position.x)
        lis.append(data.pose.pose.position.y)
        lis.append(data.pose.pose.position.z)
        lis.append(data.pose.pose.orientation.x)
        lis.append(data.pose.pose.orientation.y)
        lis.append(data.pose.pose.orientation.z)
        lis.append(data.pose.pose.orientation.w)
        self.data_odom.append(lis)

    def ekf(self):
        pass

    def save(self):
        with open('imu.csv', 'w') as f:
            write = csv.writer(f)
            write.writerow(self.fields_imu)
            write.writerows(self.data_imu)
        with open('odom.csv', 'w') as f:
            write = csv.writer(f)
            write.writerow(self.fields_odom)
            write.writerows(self.data_odom)


if __name__ == "__main__":
    rospy.init_node("save_odom")
    odom = Fusion()
    rospy.spin()
    while not rospy.is_shutdown():
        odom.update()
        # rospy.sleep(1)
    odom.save()