#!/usr/bin/env python3
import rospy, sys, math, numpy as np
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from rrt_star import RRTStar
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from drone_delivery.msg import ThreatArray
from mavros_msgs.msg import State
from cv_bridge import CvBridge

class LocalPlannerNode:
    def __init__(self):
        rospy.init_node('local_planner_node')
        self.bridge = CvBridge()
        self.global_path = []
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_waypoint_idx = 0
        self.threats = []
        self.mavros_state = State()
        self.takeoff_complete = False
        self.takeoff_altitude = 15.0
        self.waypoint_radius = 3.0
        self.max_speed = 2.0

        # Depth-based avoidance state
        self.obstacle_ahead = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_above = False
        self.min_front_dist = 999.0
        self.avoidance_active = False
        self.avoidance_vx = 0.0
        self.avoidance_vy = 0.0
        self.avoidance_vz = 0.0
        self.SAFE_DIST = 8.0   # meters - start avoiding
        self.STOP_DIST = 4.0   # meters - stop completely

        rospy.Subscriber('/planned_path', Path, self.path_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/airsim_node/drone_1/odom_local_ned', 
                        __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry,
                        self.odom_cb, queue_size=1)
        rospy.Subscriber('/predicted_threats', ThreatArray, self.threats_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/airsim_node/drone_1/front_center_custom/DepthPerspective',
                        Image, self.depth_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo('Local planner node ready.')

    def path_cb(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                           for p in msg.poses]
        self.current_waypoint_idx = 0
        rospy.loginfo(f'Received path: {len(self.global_path)} waypoints')

    def pose_cb(self, msg):
        self.current_pos = [msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z]

    def odom_cb(self, msg):
        # use AirSim odometry for altitude since EKF2 local pos not available
        p = msg.pose.pose.position
        if self.current_pos == [0.0, 0.0, 0.0]:
            self.current_pos = [p.x, p.y, -p.z]  # NED to ENU z
        else:
            self.current_pos[2] = -p.z  # just update altitude

    def threats_cb(self, msg):
        self.threats = msg.threats

    def state_cb(self, msg):
        self.mavros_state = msg

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth.shape
            
            # Divide into zones
            front  = depth[h//3:2*h//3, w//3:2*w//3]
            left   = depth[h//3:2*h//3, :w//3]
            right  = depth[h//3:2*h//3, 2*w//3:]
            top    = depth[:h//3, w//3:2*w//3]

            def zone_min(zone):
                valid = zone[(zone > 0.1) & (zone < 100)]
                return valid.min() if len(valid) > 0 else 999.0

            front_min = zone_min(front)
            left_min  = zone_min(left)
            right_min = zone_min(right)
            top_min   = zone_min(top)

            self.min_front_dist = front_min
            self.obstacle_ahead = front_min < self.SAFE_DIST
            self.obstacle_left  = left_min  < self.SAFE_DIST
            self.obstacle_right = right_min < self.SAFE_DIST
            self.obstacle_above = top_min   < 3.0

            if self.obstacle_ahead:
                rospy.logwarn_throttle(1, f'Obstacle ahead: {front_min:.1f}m L:{left_min:.1f}m R:{right_min:.1f}m')
                
                # choose avoidance direction
                if front_min < self.STOP_DIST:
                    self.avoidance_vx = -1.0  # back up
                else:
                    self.avoidance_vx = 0.0   # just stop forward

                # steer toward more open side
                if left_min > right_min:
                    self.avoidance_vy = -2.0  # go left
                else:
                    self.avoidance_vy = 2.0   # go right

                # go up if blocked on sides too
                if self.obstacle_left and self.obstacle_right:
                    self.avoidance_vz = 2.0
                    self.avoidance_vy = 0.0
                else:
                    self.avoidance_vz = 0.5

                self.avoidance_active = True
            else:
                self.avoidance_active = False
                self.avoidance_vx = 0.0
                self.avoidance_vy = 0.0
                self.avoidance_vz = 0.0

        except Exception as e:
            rospy.logerr_throttle(5, f'Depth error: {e}')

    def distance_to_waypoint(self, wp):
        dx = wp[0] - self.current_pos[0]
        dy = wp[1] - self.current_pos[1]
        return math.sqrt(dx*dx + dy*dy)

    def control_loop(self, event):
        cmd = Twist()

        rospy.loginfo_throttle(2, f'Armed:{self.mavros_state.armed} Mode:{self.mavros_state.mode} TakeoffDone:{self.takeoff_complete} Alt:{self.current_pos[2]:.1f}')
        if not self.mavros_state.armed:
            self.vel_pub.publish(cmd)
            return

        # TAKEOFF PHASE - climb straight up, ignore everything
        if not self.takeoff_complete:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 5.0  # maximum climb rate
            self.vel_pub.publish(cmd)
            alt = abs(self.current_pos[2])
            rospy.loginfo_throttle(1, f'TAKEOFF: alt={alt:.1f}m')
            if alt >= self.takeoff_altitude - 1.0:
                self.takeoff_complete = True
                rospy.loginfo('Takeoff complete!')
            return

        # COLLISION AVOIDANCE ONLY ACTIVE AFTER TAKEOFF
        if self.avoidance_active and self.takeoff_complete:
            cmd.linear.x = self.avoidance_vx
            cmd.linear.y = self.avoidance_vy
            cmd.linear.z = self.avoidance_vz
            self.vel_pub.publish(cmd)
            return

        # NAVIGATION
        if not self.global_path:
            cmd.linear.z = 0.0
            self.vel_pub.publish(cmd)
            return

        # advance waypoint
        while self.current_waypoint_idx < len(self.global_path):
            wp = self.global_path[self.current_waypoint_idx]
            if self.distance_to_waypoint(wp) < self.waypoint_radius:
                self.current_waypoint_idx += 1
                rospy.loginfo(f'Reached waypoint {self.current_waypoint_idx}')
            else:
                break

        if self.current_waypoint_idx >= len(self.global_path):
            rospy.loginfo_throttle(5, 'MISSION COMPLETE - hovering at destination')
            self.vel_pub.publish(cmd)
            return

        # move toward waypoint
        wp = self.global_path[self.current_waypoint_idx]
        dx = wp[0] - self.current_pos[0]
        dy = wp[1] - self.current_pos[1]
        dz = self.takeoff_altitude - self.current_pos[2]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist > 0.5:
            speed = min(self.max_speed, dist * 0.3)
            cmd.linear.x = (dx/dist) * speed
            cmd.linear.y = (dy/dist) * speed
        cmd.linear.z = max(-1.0, min(1.0, dz * 0.3))

        self.vel_pub.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        LocalPlannerNode().run()
    except rospy.ROSInterruptException:
        pass
