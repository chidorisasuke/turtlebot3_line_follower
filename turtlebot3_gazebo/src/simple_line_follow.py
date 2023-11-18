import rospy
from robot import Robot

if __name__ == "__main__":
    rospy.init_node('simple_line_follow')
    robot = Robot()
    try:
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