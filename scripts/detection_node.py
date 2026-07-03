#!/usr/bin/env python3
import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D
from ultralytics import YOLO

# ── class definitions ─────────────────────────────────────────────────────────
VISDRONE_CLASSES = {
    0: "pedestrian", 1: "person", 2: "bicycle", 3: "car",
    4: "van", 5: "truck", 6: "tricycle", 7: "awning-tricycle",
    8: "bus", 9: "motorcycle"
}
REMAP = {
    0: "person", 1: "person", 2: "vehicle", 3: "vehicle",
    4: "vehicle", 5: "vehicle", 6: "vehicle", 7: "vehicle",
    8: "vehicle", 9: "vehicle"
}

class DetectionNode:
    def __init__(self):
        rospy.init_node("detection_node", anonymous=False)

        # ── params ────────────────────────────────────────────────────────────
        model_path = rospy.get_param("~model_path",
            "/home/javier/catkin_ws/src/drone_delivery/weights/visdrone_yolov8n_best.pt")
        camera_topic = rospy.get_param("~camera_topic",
            "/airsim_node/drone_1/front_center_custom/Scene")
        self.conf = rospy.get_param("~confidence", 0.25)

        # ── model ─────────────────────────────────────────────────────────────
        rospy.loginfo(f"Loading model from {model_path}")
        self.model = YOLO(model_path)
        rospy.loginfo("Model loaded.")

        # ── ros ───────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/detections", Detection2DArray, queue_size=10)
        self.sub = rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        # ── timing ────────────────────────────────────────────────────────────
        self.frame_count = 0
        self.total_time  = 0.0
        rospy.loginfo(f"Subscribed to {camera_topic}")
        rospy.loginfo("Detection node ready.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")
            return

        t0 = time.time()
        results = self.model(frame, conf=self.conf, verbose=False)[0]
        elapsed_ms = (time.time() - t0) * 1000

        self.frame_count += 1
        self.total_time  += elapsed_ms

        # ── build Detection2DArray ────────────────────────────────────────────
        det_array = Detection2DArray()
        det_array.header = msg.header

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf   = float(box.conf[0])
            x1, y1, x2, y2 = map(float, box.xyxy[0])

            det = Detection2D()
            det.header = msg.header

            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.x = (x1 + x2) / 2
            bbox.center.y = (y1 + y2) / 2
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            det.bbox = bbox

            # store class info in source_img field label (simple approach)
            det.source_img = msg  # keep reference
            det_array.detections.append(det)

        self.pub.publish(det_array)

        # ── log every 30 frames ───────────────────────────────────────────────
        if self.frame_count % 30 == 0:
            avg_fps = 1000 / (self.total_time / self.frame_count)
            rospy.loginfo(f"Frame {self.frame_count} | {elapsed_ms:.1f}ms | avg {avg_fps:.1f} FPS | detections: {len(results.boxes)}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = DetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
