#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

class FailsafeNode:
    def __init__(self):
        rospy.init_node('failsafe_node')
        self.state = State()
        self.offboard_drops = 0
        
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        self.hover_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            Twist, queue_size=1)
        
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        rospy.Timer(rospy.Duration(0.05), self.monitor)
        rospy.loginfo('Failsafe node ready.')

    def state_cb(self, msg):
        if self.state.mode == 'OFFBOARD' and msg.mode != 'OFFBOARD':
            self.offboard_drops += 1
            rospy.logwarn(f'OFFBOARD dropped! Total drops: {self.offboard_drops}')
            self.set_mode(custom_mode='OFFBOARD')
        self.state = msg

    def monitor(self, event):
        if not self.state.connected:
            rospy.logwarn_throttle(5, 'MAVROS disconnected!')
            return
        if self.state.armed and self.state.mode != 'OFFBOARD':
            hover = Twist()
            self.hover_pub.publish(hover)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        FailsafeNode().run()
    except rospy.ROSInterruptException:
        pass
