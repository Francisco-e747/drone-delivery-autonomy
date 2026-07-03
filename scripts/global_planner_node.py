#!/usr/bin/env python3
import rospy, sys, time
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from rrt_star import RRTStar
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from drone_delivery.msg import ThreatArray

class GlobalPlannerNode:
    def __init__(self):
        rospy.init_node('global_planner_node')
        self.threats = []
        self.past_paths = []
        self.current_pos = (0, 0)
        self.entropy_weight = rospy.get_param('~entropy_weight', 0.3)
        rospy.Subscriber('/predicted_threats', ThreatArray, self.threats_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        rospy.loginfo('Global planner node ready.')
        rospy.Timer(rospy.Duration(5.0), self.plan_test_path)

    def threats_cb(self, msg):
        # inflate threat sphere radius = 3 * uncertainty_radius, decay over 600ms
        now = rospy.Time.now().to_sec()
        self.threats = [
            (t.predicted_position.x, t.predicted_position.y,
             max(1.0, t.uncertainty_radius * 3.0))
            for t in msg.threats
        ]

    def pose_cb(self, msg):
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)

    def plan_path(self, goal_x, goal_y):
        obstacles = self.threats + [(10, 10, 3), (20, 20, 3), (-10, 10, 3)]
        rrt = RRTStar(start=self.current_pos, goal=(goal_x, goal_y),
                      obstacles=obstacles, bounds=(-50, 50, -50, 50),
                      max_iter=2000, entropy_weight=self.entropy_weight)
        for p in self.past_paths:
            rrt.add_past_path(p)
        path, elapsed = rrt.plan()
        if path:
            self.past_paths.append(path)
            if len(self.past_paths) > 10:
                self.past_paths.pop(0)
        return path, elapsed

    def plan_test_path(self, event):
        rospy.loginfo('Planning test path...')
        path, elapsed = self.plan_path(30, 30)
        if path:
            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = rospy.Time.now()
            for x, y in path:
                pose = PoseStamped()
                pose.header = ros_path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 5.0
                pose.pose.orientation.w = 1.0
                ros_path.poses.append(pose)
            self.path_pub.publish(ros_path)
            rospy.loginfo(f'Published path: {len(path)} waypoints in {elapsed:.1f}ms')
        else:
            rospy.logwarn('No path found!')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        GlobalPlannerNode().run()
    except rospy.ROSInterruptException:
        pass
