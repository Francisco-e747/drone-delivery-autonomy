#!/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class DepthToCloud:
    def __init__(self):
        rospy.init_node("depth_to_cloud")
        self.bridge = CvBridge()
        self.camera_info = None
        
        rospy.Subscriber("/airsim_node/drone_1/front_center_custom/DepthPerspective/camera_info",
                        CameraInfo, self.info_cb)
        rospy.Subscriber("/airsim_node/drone_1/front_center_custom/DepthPerspective",
                        Image, self.depth_cb, queue_size=1)
        self.pub = rospy.Publisher("/depth_cloud", PointCloud2, queue_size=1)
        rospy.loginfo("Depth to cloud node ready.")

    def info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        if self.camera_info is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            rospy.logerr(f"Bridge error: {e}")
            return

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        h, w = depth.shape
        points = []
        step = 4  # downsample
        for v in range(0, h, step):
            for u in range(0, w, step):
                z = float(depth[v, u])
                z = z / 100.0  # convert cm to meters
                if z <= 0 or z > 100:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        cloud = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud)
        rospy.loginfo(f"Published cloud with {len(points)} points")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        DepthToCloud().run()
    except rospy.ROSInterruptException:
        pass
