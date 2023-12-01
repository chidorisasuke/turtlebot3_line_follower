import rospy
from std_msgs.msg import UInt8MultiArray, Int8MultiArray

sensor_garis = []

def line_sensor_callback(msg : UInt8MultiArray):
    global sensor_garis
    sensor_garis = [int(i) for i in msg.data]
    

if __name__ == "__main__":
    rospy.init_node("yok_main_line_follower")
    sub = rospy.Subscriber("/line_sensor", UInt8MultiArray, line_sensor_callback)
    pub_motor = rospy.Publisher("/motor", Int8MultiArray, queue_size=1)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(f"sensor_garis: {sensor_garis}")
        motor = Int8MultiArray()
        motor.data = [10,10]
        pub_motor.publish(motor)
        r.sleep()