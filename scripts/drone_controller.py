#!/usr/bin/env python3
import rospy, time, math
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        self.bridge = CvBridge()
        self.rate = rospy.Rate(20)
        self.mavros_state = State()
        self.phase = 'INIT'
        self.SAFE_DIST = 8.0
        self.STOP_DIST = 4.0
        self.obstacle_detected = False
        self.avoid_thrust = 0.55
        self.avoid_roll = 0.0
        self.goal_bearing = None
        self.should_land = False
        self._dist_to_goal = 500

        self.att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',
                                      AttitudeTarget, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/mission/takeoff_complete',
                                          Bool, queue_size=1)

        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/mission/goal',
                        __import__('sensor_msgs.msg', fromlist=['NavSatFix']).NavSatFix,
                        self.mission_goal_cb, queue_size=1)
        self._goal_gps_lat = None
        self._goal_gps_lon = None
        self._current_lat = None
        self._current_lon = None
        rospy.Subscriber('/mavros/global_position/raw/gps_vel',
                        __import__('geometry_msgs.msg', fromlist=['TwistStamped']).TwistStamped,
                        self.vel_cb, queue_size=1)
        rospy.Subscriber('/mission/hover',
                        __import__('std_msgs.msg', fromlist=['Bool']).Bool,
                        self.hover_cb, queue_size=1)
        self._hovering = False
        self._hover_count = 0
        self._current_yaw = 0.0
        self._goal_yaw = None
        rospy.Subscriber('/airsim_node/drone_1/front_center_custom/DepthPerspective',
                        Image, self.depth_cb, queue_size=1)
        rospy.Subscriber('/mission/goal_local', Point, self.goal_local_cb, queue_size=1)
        rospy.Subscriber('/mission/land', Bool, self.land_cb, queue_size=1)
        rospy.Subscriber('/mavros/global_position/raw/fix',
                        __import__('sensor_msgs.msg', fromlist=['NavSatFix']).NavSatFix,
                        self.fix_cb, queue_size=1)
        self.current_altitude = 0.0
        self.target_altitude = 290.0

        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo('Drone controller ready.')

    def state_cb(self, msg):
        self.mavros_state = msg

    def hover_cb(self, msg):
        if msg.data:
            self._hovering = True
            self._hover_count = 0
            rospy.loginfo('Hovering to recalibrate...')

    def mission_goal_cb(self, msg):
        self._goal_gps_lat = msg.latitude
        self._goal_gps_lon = msg.longitude

    def vel_cb(self, msg):
        import math
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        if math.sqrt(vx**2 + vy**2) > 0.5:
            self._current_yaw = math.atan2(vx, vy)
        # recompute goal_yaw from fixed goal GPS and current GPS
        if self._goal_gps_lat and hasattr(self, "_current_lat"):
            dlat = self._goal_gps_lat - self._current_lat
            dlon = self._goal_gps_lon - self._current_lon
            x_east = dlon * math.cos(math.radians(self._current_lat)) * 111320
            y_north = dlat * 111320
            self._goal_yaw = math.atan2(x_east, y_north)  # ENU bearing
            rospy.loginfo_throttle(2, f'GoalYaw={math.degrees(self._goal_yaw):.0f} CurrYaw={math.degrees(self._current_yaw):.0f} dx={x_east:.0f} dy={y_north:.0f}')

    def goal_local_cb(self, msg):
        bearing = math.atan2(msg.x, msg.y)
        if self.goal_bearing is None:
            self.goal_bearing = bearing
        else:
            self.goal_bearing = 0.95 * self.goal_bearing + 0.05 * bearing
        self._dist_to_goal = msg.z if msg.z > 0 else 500
        self._goal_yaw = self.goal_bearing

    def land_cb(self, msg):
        if msg.data:
            self.should_land = True
            rospy.loginfo('Land command received')

    def fix_cb(self, msg):
        self.current_altitude = msg.altitude
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude

    def depth_cb(self, msg):
        if self.phase not in ['TAKEOFF', 'NAVIGATE']:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth.shape
            front = depth[h//3:2*h//3, w//3:2*w//3]
            left  = depth[h//3:2*h//3, :w//3]
            right = depth[h//3:2*h//3, 2*w//3:]
            upper = depth[:h//3, w//4:3*w//4]

            def zone_min(z):
                v = z[(z > 0.5) & (z < 100)]
                return v.min() if len(v) > 0 else 999.0

            f = zone_min(front)
            l = zone_min(left)
            r = zone_min(right)
            u = zone_min(upper)

            if self.phase == 'TAKEOFF' and self.current_alt() < 10.0:
                self.obstacle_detected = False
                return

            if f < self.SAFE_DIST:
                self.obstacle_detected = True
                if u > 5.0:
                    self.avoid_thrust = 0.75
                    self.avoid_roll = 0.0
                elif l > r:
                    self.avoid_thrust = 0.58
                    self.avoid_roll = -0.15
                else:
                    self.avoid_thrust = 0.58
                    self.avoid_roll = 0.15
                rospy.logwarn_throttle(1, f'Obstacle: F={f:.1f}m L={l:.1f}m R={r:.1f}m U={u:.1f}m')
            else:
                self.obstacle_detected = False
                self.avoid_thrust = 0.528
                self.avoid_roll = 0.0
        except:
            pass

    def current_alt(self):
        try:
            fix = rospy.wait_for_message('/mavros/global_position/raw/fix',
                                        __import__('sensor_msgs.msg', fromlist=['NavSatFix']).NavSatFix,
                                        timeout=0.1)
            return fix.altitude - 100.0  # approx above ground
        except:
            return 0.0

    def publish_attitude(self, thrust=0.528, roll=0.0, pitch=0.0, yaw=None):
        import tf.transformations as tft
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = 0b00000111
        yaw_val = yaw if yaw is not None else self._current_yaw
        q = tft.quaternion_from_euler(roll, pitch, yaw_val)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        msg.thrust = thrust
        self.att_pub.publish(msg)

    def run(self):
        # INIT
        rospy.loginfo('PHASE: INIT')
        self.phase = 'INIT'
        t0 = time.time()
        while time.time() - t0 < 10.0 and not rospy.is_shutdown():
            self.publish_attitude()
            self.rate.sleep()

        # ARM
        rospy.loginfo('PHASE: ARMING')
        self.phase = 'ARMING'
        for _ in range(20):
            self.publish_attitude()
            self.set_mode_client(custom_mode='OFFBOARD')
            self.rate.sleep()
        for _ in range(30):
            self.publish_attitude()
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo('Armed!')
                break
            self.rate.sleep()

        # TAKEOFF
        rospy.loginfo('PHASE: TAKEOFF')
        self.phase = 'TAKEOFF'
        t0 = time.time()
        while time.time() - t0 < 25.0 and not rospy.is_shutdown():
            if self.mavros_state.mode != 'OFFBOARD':
                self.set_mode_client(custom_mode='OFFBOARD')
            self.publish_attitude(thrust=0.60)  # high thrust to climb fast past buildings
            self.rate.sleep()

        # NAVIGATE
        rospy.loginfo('PHASE: NAVIGATE')
        self.phase = 'NAVIGATE'
        self.takeoff_pub.publish(Bool(data=True))
        while not rospy.is_shutdown():
            if self.should_land:
                rospy.loginfo('Landing - hovering then descending...')
                # hover in place for 3 seconds first
                for i in range(60):
                    self.publish_attitude(thrust=0.528, roll=0.0)
                    self.rate.sleep()
                # gradual descent
                rospy.loginfo('Descending...')
                for i in range(300):
                    thrust = max(0.38, 0.525 - i*0.0005)
                    self.publish_attitude(thrust=thrust, roll=0.0)
                    self.rate.sleep()
                self.set_mode_client(custom_mode='AUTO.LAND')
                return
            if self.mavros_state.mode != 'OFFBOARD':
                self.set_mode_client(custom_mode='OFFBOARD')
            if self.obstacle_detected:
                self.publish_attitude(thrust=self.avoid_thrust, roll=self.avoid_roll)
            elif self.goal_bearing is not None:
                if self._hovering:
                    self._hover_count += 1
                    alt_error = self.target_altitude - self.current_altitude
                    thrust = max(0.50, min(0.56, 0.528 + alt_error * 0.001))
                    self.publish_attitude(thrust=thrust, roll=0.0)
                    if self._hover_count > 100:  # hover 5 seconds then resume
                        self._hovering = False
                        rospy.loginfo('Resuming navigation...')
                else:
                    import math
                    dist = getattr(self, '_dist_to_goal', 999)
                    alt_error = self.target_altitude - self.current_altitude
                    thrust = max(0.50, min(0.56, 0.528 + alt_error * 0.001))
                    if dist < 20:
                        self.publish_attitude(thrust=thrust)
                    elif getattr(self, '_goal_yaw', None) is not None:
                        hdg_err = self._goal_yaw - self._current_yaw
                        while hdg_err > math.pi: hdg_err -= 2*math.pi
                        while hdg_err < -math.pi: hdg_err += 2*math.pi
                        if abs(hdg_err) > 0.15:
                            self.publish_attitude(thrust=thrust, yaw=self._goal_yaw)
                        else:
                            self.publish_attitude(thrust=thrust, pitch=-0.03, yaw=self._goal_yaw)
                        rospy.loginfo_throttle(3, f'Dist={dist:.0f}m HdgErr={math.degrees(hdg_err):.0f}')
                    else:
                        roll = max(-0.08, min(0.08, -self.goal_bearing * 0.15))
                        self.publish_attitude(thrust=thrust, roll=roll)
            else:
                alt_error = self.target_altitude - self.current_altitude
                thrust = max(0.50, min(0.56, 0.528 + alt_error * 0.001))
                self.publish_attitude(thrust=thrust)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DroneController().run()
    except rospy.ROSInterruptException:
        pass
