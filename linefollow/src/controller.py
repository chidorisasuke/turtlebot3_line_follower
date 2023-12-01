import rospy
from linefollow.Robot_API import Robot
from std_msgs.msg import UInt8MultiArray, Int8MultiArray

def publish_cmd_vel(msg):
    robot.motor(msg.data[0], msg.data[1])

if __name__ == "__main__":
    rospy.init_node("Line_Follower_API")
    robot = Robot()
    r = rospy.Rate(60)

    if rospy.has_param("/line_color"):
        line_color = rospy.get_param("/line_color")
    else:
        rospy.set_param("/line_color", True)
        line_color = True

    sub_motor = rospy.Subscriber("/motor", Int8MultiArray, publish_cmd_vel)
    pub_sensor_line = rospy.Publisher("/line_sensor", UInt8MultiArray, queue_size=1)
    last = rospy.Time.now()
    while not rospy.is_shutdown():
            if rospy.Time.now() - last > rospy.Duration(5):
                rospy.loginfo("[Robot API] Heartbeat every 5 seconds")
                last = rospy.Time.now()
            lines = UInt8MultiArray()
            lines.data = robot.get_lines(line_black=line_color)
            pub_sensor_line.publish(lines)
            r.sleep()

