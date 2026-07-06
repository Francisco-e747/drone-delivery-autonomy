#!/usr/bin/env python3
"""
Unified drone startup node:
1. Publishes attitude setpoints to satisfy OFFBOARD requirement
2. Arms the drone and switches to OFFBOARD mode
3. Monitors altitude via AirSim odometry
4. Once at cruise altitude, switches to velocity setpoints for navigation
5. Activates collision avoidance
"""
import rospy
import time
import math
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Path, Odometry
from drone_delivery.msg import ThreatArray
from cv_bridge import CvBridge

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        self.bridge = CvBridge()
        self.rate = rospy.Rate(20)

        # State
        self.mavros_state = State()
        self.current_pos = [0.0, 0.0, 0.0]
        self.global_path = []
        self.current_waypoint_idx = 0
        self.phase = 'INIT'  # INIT -> ARMING -> TAKEOFF -> NAVIGATE
        self.cruise_altitude = 15.0
        self.waypoint_radius = 3.0
        self.max_speed = 2.0

        # Avoidance
        self.avoidance_active = False
        self.avoidance_vx = 0.0
        self.avoidance_vy = 0.0
        self.avoidance_vz = 0.0
        self.SAFE_DIST = 8.0
        self.STOP_DIST = 4.0

        # Publishers
        self.att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',
                                       AttitudeTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',
                                       Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/airsim_node/drone_1/odom_local_ned',
                        Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/airsim_node/drone_1/front_center_custom/DepthPerspective',
                        Image, self.depth_cb, queue_size=1)
        rospy.Subscriber('/planned_path', Path, self.path_cb, queue_size=1)

        # Services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo('Drone controller ready.')

    def state_cb(self, msg):
        self.mavros_state = msg

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_pos = [p.x, p.y, abs(p.z)]

    def path_cb(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y)
                           for p in msg.poses]
        self.current_waypoint_idx = 0
        rospy.loginfo(f'Path received: {len(self.global_path)} waypoints')

    def depth_cb(self, msg):
        if self.phase != 'NAVIGATE':
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth.shape
            front = depth[h//3:2*h//3, w//3:2*w//3]
            left  = depth[h//3:2*h//3, :w//3]
            right = depth[h//3:2*h//3, 2*w//3:]

            def zone_min(z):
                v = z[(z > 0.5) & (z < 100)]
                return v.min() if len(v) > 0 else 999.0

            f = zone_min(front)
            l = zone_min(left)
            r = zone_min(right)

            if f < self.SAFE_DIST:
                self.avoidance_active = True
                self.avoidance_vx = -1.0 if f < self.STOP_DIST else 0.0
                self.avoidance_vy = -3.0 if l > r else 3.0
                self.avoidance_vz = 2.0
                rospy.logwarn_throttle(1, f'Obstacle: front={f:.1f}m L={l:.1f}m R={r:.1f}m')
            else:
                self.avoidance_active = False
                self.avoidance_vx = self.avoidance_vy = self.avoidance_vz = 0.0
        except:
            pass

    def publish_attitude(self):
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = 7
        msg.orientation.w = 1.0
        msg.thrust = 0.5
        self.att_pub.publish(msg)

    def distance_to_waypoint(self, wp):
        dx = wp[0] - self.current_pos[0]
        dy = wp[1] - self.current_pos[1]
        return math.sqrt(dx*dx + dy*dy)

    def run(self):
        # Phase 1: publish attitude setpoints for 10s
        rospy.loginfo('PHASE: INIT - publishing attitude setpoints...')
        self.phase = 'INIT'
        t0 = time.time()
        while time.time() - t0 < 10.0 and not rospy.is_shutdown():
            self.publish_attitude()
            self.rate.sleep()

        # Phase 2: switch to OFFBOARD and arm
        rospy.loginfo('PHASE: ARMING')
        self.phase = 'ARMING'
        for _ in range(20):
            self.publish_attitude()
            self.set_mode_client(custom_mode='OFFBOARD')
            self.rate.sleep()

        for _ in range(20):
            self.publish_attitude()
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo('Armed!')
                break
            self.rate.sleep()

        # Phase 3: takeoff - keep publishing attitude until cruise altitude
        rospy.loginfo('PHASE: TAKEOFF')
        self.phase = 'TAKEOFF'
        while not rospy.is_shutdown():
            alt = self.current_pos[2]
            rospy.loginfo_throttle(2, f'Altitude: {alt:.1f}m / {self.cruise_altitude}m')
            if alt >= self.cruise_altitude - 2.0:
                rospy.loginfo('Cruise altitude reached!')
                break
            self.publish_attitude()
            # keep OFFBOARD alive
            if self.mavros_state.mode != 'OFFBOARD':
                self.set_mode_client(custom_mode='OFFBOARD')
            self.rate.sleep()

        # Phase 4: navigate with velocity setpoints + collision avoidance
        rospy.loginfo('PHASE: NAVIGATE - switching to velocity control')
        self.phase = 'NAVIGATE'
        while not rospy.is_shutdown():
            cmd = Twist()

            if self.avoidance_active:
                cmd.linear.x = self.avoidance_vx
                cmd.linear.y = self.avoidance_vy
                cmd.linear.z = self.avoidance_vz
                self.vel_pub.publish(cmd)
                self.rate.sleep()
                continue

            alt = self.current_pos[2]
            dz = self.cruise_altitude - alt
            cmd.linear.z = max(-1.0, min(1.0, dz * 0.3))

            if not self.global_path:
                self.vel_pub.publish(cmd)
                self.rate.sleep()
                continue

            # advance waypoint
            while self.current_waypoint_idx < len(self.global_path):
                wp = self.global_path[self.current_waypoint_idx]
                if self.distance_to_waypoint(wp) < self.waypoint_radius:
                    self.current_waypoint_idx += 1
                    rospy.loginfo(f'Waypoint {self.current_waypoint_idx} reached!')
                else:
                    break

            if self.current_waypoint_idx >= len(self.global_path):
                rospy.loginfo_throttle(5, 'MISSION COMPLETE - hovering at destination')
                self.vel_pub.publish(cmd)
                self.rate.sleep()
                continue

            wp = self.global_path[self.current_waypoint_idx]
            dx = wp[0] - self.current_pos[0]
            dy = wp[1] - self.current_pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 0.5:
                speed = min(self.max_speed, dist * 0.3)
                cmd.linear.x = (dx/dist) * speed
                cmd.linear.y = (dy/dist) * speed

            self.vel_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DroneController().run()
    except rospy.ROSInterruptException:
        pass
