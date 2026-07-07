#!/usr/bin/env python3
"""
Mission Manager for autonomous drone delivery.
- Auto-detects current GPS position (Point A)
- Accepts user-defined destination via ROS topic (Point B)
- Plans RRT* path from A to B
- Sends waypoints to drone_controller
- Signals auto-land when destination reached
"""
import rospy
import sys
import math
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from rrt_star import RRTStar
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')

        # GPS state
        self.current_gps = None
        self.origin_gps = None  # Point A - set once on first fix
        self.goal_gps = None    # Point B - set by user

        # Mission state
        self.mission_active = False
        self.pending_goal = None
        self.takeoff_complete = False
        self.waypoints = []
        self.current_wp_idx = 0
        self.current_local = [0.0, 0.0]
        self.WAYPOINT_RADIUS = 15.0  # meters

        # Subscribers
        rospy.Subscriber('/mavros/global_position/raw/fix',
                        NavSatFix, self.gps_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose',
                        PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/mission/goal',
                        NavSatFix, self.goal_cb, queue_size=1)
        rospy.Subscriber('/mavros/state',
                        State, self.state_cb, queue_size=1)
        rospy.Subscriber('/mission/takeoff_complete',
                        __import__('std_msgs.msg', fromlist=['Bool']).Bool,
                        self.takeoff_cb, queue_size=1)

        # Publishers
        self.goal_local_pub = rospy.Publisher('/mission/goal_local',
                                            __import__('geometry_msgs.msg', fromlist=['Point']).Point,
                                            queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/planned_path', Path,
                                       queue_size=1, latch=True)
        self.land_pub = rospy.Publisher('/mission/land', Bool, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)

        self.mavros_state = State()
        rospy.loginfo('Mission manager ready.')
        rospy.loginfo('Set destination: rostopic pub /mission/goal sensor_msgs/NavSatFix "{latitude: XX.XXX, longitude: YY.YYY, altitude: 0.0}" -1')

    def state_cb(self, msg):
        self.mavros_state = msg

    def takeoff_cb(self, msg):
        if msg.data and not self.takeoff_complete:
            self.takeoff_complete = True
            rospy.loginfo('Takeoff complete signal received')
            if self.pending_goal is not None:
                from geometry_msgs.msg import Point
                gp = Point()
                gp.x = self.pending_goal[0]
                gp.y = self.pending_goal[1]
                self.goal_local_pub.publish(gp)
                rospy.loginfo(f'Goal sent: x={gp.x:.1f}m y={gp.y:.1f}m')

    def gps_cb(self, msg):
        self.current_gps = msg
        if self.origin_gps is None and msg.status.status >= 0:
            self.origin_gps = msg
            rospy.loginfo(f'Point A set: lat={msg.latitude:.6f} lon={msg.longitude:.6f}')

    def pose_cb(self, msg):
        self.current_local = [msg.pose.position.x, msg.pose.position.y]

    def gps_to_local(self, lat, lon):
        """Convert GPS to local meters relative to origin"""
        if self.origin_gps is None:
            return (0.0, 0.0)
        dlat = lat - self.origin_gps.latitude
        dlon = lon - self.origin_gps.longitude
        x = dlon * math.cos(math.radians(self.origin_gps.latitude)) * 111320
        y = dlat * 111320
        return (x, y)

    def local_to_gps(self, x, y):
        """Convert local meters to GPS"""
        if self.origin_gps is None:
            return (0.0, 0.0)
        dlat = y / 111320
        dlon = x / (111320 * math.cos(math.radians(self.origin_gps.latitude)))
        return (self.origin_gps.latitude + dlat,
                self.origin_gps.longitude + dlon)

    def goal_cb(self, msg):
        self.goal_gps = msg
        rospy.loginfo(f'Point B set: lat={msg.latitude:.6f} lon={msg.longitude:.6f}')
        self.plan_mission()

    def plan_mission(self):
        if self.origin_gps is None:
            rospy.logwarn('No GPS fix yet - cannot plan mission')
            return
        if self.goal_gps is None:
            rospy.logwarn('No goal set')
            return

        # Convert to local coordinates
        start = self.gps_to_local(self.origin_gps.latitude,
                                   self.origin_gps.longitude)
        goal  = self.gps_to_local(self.goal_gps.latitude,
                                   self.goal_gps.longitude)

        dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)
        rospy.loginfo(f'Planning path: A={start} B={goal} distance={dist:.0f}m')

        # RRT* planning
        bounds = (min(start[0], goal[0]) - 50,
                  max(start[0], goal[0]) + 50,
                  min(start[1], goal[1]) - 50,
                  max(start[1], goal[1]) + 50)

        rrt = RRTStar(start=start, goal=goal,
                     obstacles=[],  # depth camera handles avoidance
                     bounds=bounds,
                     max_iter=3000,
                     entropy_weight=0.3)
        path, elapsed = rrt.plan()

        if path:
            rospy.loginfo(f'Path found: {len(path)} waypoints in {elapsed:.0f}ms')
            self.waypoints = path
            self.current_wp_idx = 0
            self.mission_active = True
            self.publish_path(path)
            self.status_pub.publish(String(data='MISSION_ACTIVE'))
            # wait for takeoff before publishing goal
            self.pending_goal = (goal[0] - start[0], goal[1] - start[1])
            rospy.loginfo(f'Goal pending until takeoff complete: x={self.pending_goal[0]:.1f}m y={self.pending_goal[1]:.1f}m')
        else:
            rospy.logwarn('No path found!')
            self.status_pub.publish(String(data='PLAN_FAILED'))

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = rospy.Time.now()
        for x, y in path:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 20.0  # cruise altitude
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)
        rospy.loginfo(f'Path published with {len(path)} waypoints')

    def check_waypoint_progress(self):
        if not self.mission_active or not self.waypoints:
            return
        if self.current_gps is None:
            return

        # check distance to final goal using GPS
        if self.goal_gps is not None:
            dlat = self.goal_gps.latitude - self.current_gps.latitude
            dlon = self.goal_gps.longitude - self.current_gps.longitude
            dist_m = math.sqrt((dlat*111320)**2 + (dlon*111320*math.cos(math.radians(self.current_gps.latitude)))**2)
            rospy.loginfo_throttle(3, f'Distance to goal: {dist_m:.0f}m')
            self.status_pub.publish(String(data=f'DISTANCE:{dist_m:.0f}m'))
            # continuously update goal_local so drone steers correctly
            from geometry_msgs.msg import Point
            gp = Point()
            gp.x = dlon * math.cos(math.radians(self.current_gps.latitude)) * 111320
            gp.y = dlat * 111320
            self.goal_local_pub.publish(gp)
            if dist_m < 100.0:  # within 100m of goal
                rospy.loginfo('MISSION COMPLETE - sending land command')
                self.land_pub.publish(Bool(data=True))
                self.status_pub.publish(String(data='MISSION_COMPLETE'))
                self.mission_active = False
        self.pending_goal = None
        self.takeoff_complete = False

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.check_waypoint_progress()
            rate.sleep()

if __name__ == '__main__':
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
