#!/usr/bin/env python3
import rospy, time, math
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        self.bridge = CvBridge()
        self.rate = rospy.Rate(20)
        self.mavros_state = State()
        self.phase = 'INIT'
        self.thrust = 0.55  # hover thrust
        self.roll = 0.0
        self.pitch = 0.0
        self.SAFE_DIST = 8.0
        self.STOP_DIST = 4.0
        self.obstacle_detected = False
        self.avoid_thrust = 0.55
        self.avoid_roll = 0.0
        self.goal_bearing = None
        self.should_land = False

        self.att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',
                                      AttitudeTarget, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/mission/goal_local',
                        __import__('geometry_msgs.msg', fromlist=['Point']).Point,
                        self.goal_local_cb, queue_size=1)
        rospy.Subscriber('/mission/land',
                        __import__('std_msgs.msg', fromlist=['Bool']).Bool,
                        self.land_cb, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/mission/takeoff_complete',
                        __import__('std_msgs.msg', fromlist=['Bool']).Bool,
                        queue_size=1)
        rospy.Subscriber('/mavros/global_position/global',
                        __import__('sensor_msgs.msg', fromlist=['NavSatFix']).NavSatFix,
                        self.gps_cb, queue_size=1)
        self.gps_pos = None
        rospy.Subscriber('/airsim_node/drone_1/front_center_custom/DepthPerspective',
                        Image, self.depth_cb, queue_size=1)
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo('Drone controller ready.')

    def state_cb(self, msg):
        self.mavros_state = msg

    def gps_cb(self, msg):
        self.gps_pos = msg
        if not hasattr(self, '_gps_logged'):
            rospy.loginfo(f'GPS: lat={msg.latitude:.6f} lon={msg.longitude:.6f} alt={msg.altitude:.1f}')
            self._gps_logged = True

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

            if f < self.SAFE_DIST:
                self.obstacle_detected = True
                # go up if clear above, otherwise roll away
                if u > 5.0:
                    self.avoid_thrust = 0.75  # climb
                    self.avoid_roll = 0.0
                elif l > r:
                    self.avoid_thrust = 0.58
                    self.avoid_roll = -0.15  # roll left
                else:
                    self.avoid_thrust = 0.58
                    self.avoid_roll = 0.15   # roll right
                rospy.logwarn_throttle(1, f'Obstacle: F={f:.1f}m L={l:.1f}m R={r:.1f}m U={u:.1f}m')
            else:
                self.obstacle_detected = False
                self.avoid_thrust = 0.55
                self.avoid_roll = 0.0
        except:
            pass

    def publish_attitude(self, thrust=0.55, roll=0.0, pitch=0.0):
        from geometry_msgs.msg import Quaternion
        import tf.transformations as tft
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = 0b00000111  # ignore body rates
        q = tft.quaternion_from_euler(roll, pitch, 0.0)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        msg.thrust = thrust
        self.att_pub.publish(msg)

    def goal_local_cb(self, msg):
        import math
        bearing = math.atan2(msg.x, msg.y)
        if self.goal_bearing is None:
            self.goal_bearing = bearing
        else:
            self.goal_bearing = 0.95 * self.goal_bearing + 0.05 * bearing

    def land_cb(self, msg):
        if msg.data:
            rospy.loginfo('Land command - AUTO.LAND')
            self.should_land = True

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
        for _ in range(20):
            self.publish_attitude()
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo('Armed!')
                break
            self.rate.sleep()

        # TAKEOFF - 25 seconds
        rospy.loginfo('PHASE: TAKEOFF')
        self.phase = 'TAKEOFF'
        t0 = time.time()
        while time.time() - t0 < 25.0 and not rospy.is_shutdown():
            if self.mavros_state.mode != 'OFFBOARD':
                self.set_mode_client(custom_mode='OFFBOARD')
            self.publish_attitude(thrust=0.6)
            self.rate.sleep()

        # NAVIGATE
        rospy.loginfo('PHASE: NAVIGATE')
        self.phase = 'NAVIGATE'
        from std_msgs.msg import Bool
        self.takeoff_pub.publish(Bool(data=True))
        while not rospy.is_shutdown():
            if self.should_land:
                self.set_mode_client(custom_mode='AUTO.LAND')
                return
            if self.mavros_state.mode != 'OFFBOARD':
                self.set_mode_client(custom_mode='OFFBOARD')
            if self.obstacle_detected:
                self.publish_attitude(thrust=self.avoid_thrust, roll=self.avoid_roll)
            elif self.goal_bearing is not None:
                import math
                roll = max(-0.08, min(0.08, -self.goal_bearing * 0.15))
                self.publish_attitude(thrust=0.528, roll=roll)
            else:
                self.publish_attitude(thrust=0.528)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DroneController().run()
    except rospy.ROSInterruptException:
        pass
