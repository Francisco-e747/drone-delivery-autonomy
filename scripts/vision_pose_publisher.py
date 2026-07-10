#!/usr/bin/env python3
"""
Publishes GPS position as vision pose to fix EKF2 xy_valid.
Converts GPS to local ENU coordinates and publishes at 30Hz.
"""
import rospy
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class VisionPosePublisher:
    def __init__(self):
        rospy.init_node('vision_pose_publisher')
        self.origin_gps = None
        self.current_gps = None

        rospy.Subscriber('/mavros/global_position/raw/fix',
                        NavSatFix, self.gps_cb, queue_size=1)
        rospy.Subscriber('/mavros/global_position/raw/gps_vel',
                        __import__('geometry_msgs.msg', fromlist=['TwistStamped']).TwistStamped,
                        self.vel_cb, queue_size=1)
        self.pub = rospy.Publisher('/mavros/vision_pose/pose',
                                  PoseStamped, queue_size=1)
        self.yaw = 0.0
        rospy.Timer(rospy.Duration(1.0/30.0), self.publish)
        rospy.loginfo('Vision pose publisher ready')

    def vel_cb(self, msg):
        import math
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        if speed > 0.5:
            self.yaw = math.atan2(vx, vy)

    def gps_cb(self, msg):
        if msg.status.status < 0:
            return
        self.current_gps = msg
        if self.origin_gps is None:
            self.origin_gps = msg
            rospy.loginfo(f'Origin set: lat={msg.latitude:.6f} lon={msg.longitude:.6f}')

    def gps_to_local(self, lat, lon, alt):
        if self.origin_gps is None:
            return None
        dlat = lat - self.origin_gps.latitude
        dlon = lon - self.origin_gps.longitude
        x = dlon * math.cos(math.radians(self.origin_gps.latitude)) * 111320
        y = dlat * 111320
        z = alt - self.origin_gps.altitude
        return x, y, z

    def publish(self, event):
        if self.current_gps is None:
            return
        local = self.gps_to_local(
            self.current_gps.latitude,
            self.current_gps.longitude,
            self.current_gps.altitude)
        if local is None:
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = local[0]
        msg.pose.position.y = local[1]
        msg.pose.position.z = local[2]
        import tf.transformations as tft
        import math
        q = tft.quaternion_from_euler(0, 0, self.yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        VisionPosePublisher().run()
    except rospy.ROSInterruptException:
        pass
