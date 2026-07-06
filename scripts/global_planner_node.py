#!/usr/bin/env python3
import rospy, sys, time
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from rrt_star import RRTStar
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from drone_delivery.msg import ThreatArray
from mavros_msgs.msg import State

class GlobalPlannerNode:
    def __init__(self):
        rospy.init_node('global_planner_node')
        self.threats = []
        self.past_paths = []
        self.current_pos = (0, 0)
        self.entropy_weight = rospy.get_param('~entropy_weight', 0.3)
        self.goal_x = rospy.get_param('~goal_x', 40.0)
        self.goal_y = rospy.get_param('~goal_y', 40.0)
        self.mission_complete = False
        self.path_published = False

        rospy.Subscriber('/predicted_threats', ThreatArray, self.threats_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.state = None
        rospy.loginfo(f'Global planner ready. Goal: ({self.goal_x}, {self.goal_y})')
        rospy.Timer(rospy.Duration(2.0), self.plan_once, oneshot=True)

    def threats_cb(self, msg):
        self.threats = [(t.predicted_position.x, t.predicted_position.y, 2.0) for t in msg.threats]

    def pose_cb(self, msg):
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)

    def state_cb(self, msg):
        self.state = msg

    def plan_once(self, event):
        if self.mission_complete:
            return
        rospy.loginfo(f'Planning path from {self.current_pos} to ({self.goal_x},{self.goal_y})')
        obstacles = self.threats + [(10,10,3),(20,20,3),(-10,10,3)]
        rrt = RRTStar(start=self.current_pos, goal=(self.goal_x, self.goal_y),
                      obstacles=obstacles, bounds=(-100,100,-100,100),
                      max_iter=3000, entropy_weight=self.entropy_weight)
        for p in self.past_paths:
            rrt.add_past_path(p)
        path, elapsed = rrt.plan()
        if path:
            self.past_paths.append(path)
            if len(self.past_paths) > 10:
                self.past_paths.pop(0)
            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = rospy.Time.now()
            for x, y in path:
                pose = PoseStamped()
                pose.header = ros_path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 10.0
                pose.pose.orientation.w = 1.0
                ros_path.poses.append(pose)
            self.path_pub.publish(ros_path)
            self.path_published = True
            rospy.loginfo(f'Path published: {len(path)} waypoints in {elapsed:.1f}ms')
        else:
            rospy.logwarn('No path found, retrying in 5s')
            rospy.Timer(rospy.Duration(5.0), self.plan_once, oneshot=True)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        GlobalPlannerNode().run()
    except rospy.ROSInterruptException:
        pass
