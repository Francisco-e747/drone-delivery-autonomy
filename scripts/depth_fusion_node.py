#!/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose

class DepthFusionNode:
    def __init__(self):
        rospy.init_node("depth_fusion_node", anonymous=False)

        self.bridge = CvBridge()
        self.latest_depth = None

        # Camera intrinsics for 640x480, FOV 90 degrees
        self.fx = 320.0  # focal length x
        self.fy = 320.0  # focal length y
        self.cx = 320.0  # principal point x
        self.cy = 240.0  # principal point y

        rospy.Subscriber("/airsim_node/drone_1/front_center_custom/DepthPerspective",
                         Image, self.depth_callback, queue_size=1)
        rospy.Subscriber("/detections",
                         Detection2DArray, self.detection_callback, queue_size=1)

        self.pub = rospy.Publisher("/detections_3d", PoseArray, queue_size=10)
        rospy.loginfo("Depth fusion node ready.")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            rospy.logerr(f"Depth bridge error: {e}")

    def detection_callback(self, msg):
        if self.latest_depth is None:
            return

        pose_array = PoseArray()
        pose_array.header = msg.header

        for det in msg.detections:
            u = int(det.bbox.center.x)
            v = int(det.bbox.center.y)

            # bounds check
            h, w = self.latest_depth.shape
            if not (0 <= u < w and 0 <= v < h):
                continue

            depth = float(self.latest_depth[v, u])
            if depth <= 0 or depth > 1000:
                continue

            # project to 3D
            x = (u - self.cx) * depth / self.fx
            y = (v - self.cy) * depth / self.fy
            z = depth

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)
        if len(pose_array.poses) > 0:
            rospy.loginfo(f"Published {len(pose_array.poses)} 3D detections")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = DepthFusionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
