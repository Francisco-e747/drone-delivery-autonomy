#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import State

class SetpointPublisher:
    def __init__(self):
        rospy.init_node('setpoint_publisher_node')
        self.latest_cmd = Twist()
        self.mavros_state = State()
        
        rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped',
                        TwistStamped, self.cmd_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        
        self.pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            Twist, queue_size=1)
        
        rospy.Timer(rospy.Duration(0.05), self.publish_loop)
        rospy.loginfo('Setpoint publisher ready at 20Hz.')

    def cmd_cb(self, msg):
        self.latest_cmd = msg.twist

    def state_cb(self, msg):
        self.mavros_state = msg

    def publish_loop(self, event):
        self.pub.publish(self.latest_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        SetpointPublisher().run()
    except rospy.ROSInterruptException:
        pass
