import rospy
from linefollow.Robot_API import Robot


if __name__ == "__main__":
    rospy.init_node("line_follow")
    try:
        robot = Robot()
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            robot.motor(-10,10)
            # robot.move( 0,0.1)
            # robot.stop()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
