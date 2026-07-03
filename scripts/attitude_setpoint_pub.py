#!/usr/bin/env python3
"""
Attitude setpoint publisher for PX4 OFFBOARD mode initialization.
Publishes level hover attitude at 20Hz to satisfy PX4's offboard signal requirement.
Must be running for at least 10 seconds before OFFBOARD mode can be engaged.
"""
import rospy
from mavros_msgs.msg import AttitudeTarget

rospy.init_node('attitude_setpoint_pub')
pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
rate = rospy.Rate(20)

msg = AttitudeTarget()
msg.type_mask = 7        # ignore body rates
msg.orientation.w = 1.0  # level attitude
msg.thrust = 0.5         # 50% thrust hover

rospy.loginfo("Publishing attitude setpoints at 20Hz...")
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
