#!/usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from drone_delivery.msg import TrackedObject, TrackedObjectArray
import sys
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from tracker import SORTTracker
from predictor import ConstantVelocityPredictor
from threat_classifier import ThreatClassifier

class TrackerNode:
    def __init__(self):
        rospy.init_node("tracker_node", anonymous=False)
        self.tracker = SORTTracker(max_age=10, min_hits=2, iou_threshold=0.3)
        self.predictor = ConstantVelocityPredictor(dt=0.5)
        self.classifier = ThreatClassifier()
        self.track_history = {}  # track_id -> list of (x,y,w,h)

        rospy.Subscriber("/detections", Detection2DArray, self.detection_callback, queue_size=1)
        self.tracks_pub = rospy.Publisher("/tracked_objects", TrackedObjectArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/track_markers", MarkerArray, queue_size=10)
        rospy.loginfo("Tracker node ready.")

    def detection_callback(self, msg):
        dets = []
        for det in msg.detections:
            x = det.bbox.center.x - det.bbox.size_x / 2
            y = det.bbox.center.y - det.bbox.size_y / 2
            w = det.bbox.size_x
            h = det.bbox.size_y
            dets.append((x, y, w, h))

        tracks = self.tracker.update(dets)

        tracked_array = TrackedObjectArray()
        tracked_array.header = msg.header
        marker_array = MarkerArray()

        for i, track in enumerate(tracks):
            x, y, w, h, tid = track
            tid = int(tid)

            # update history
            if tid not in self.track_history:
                self.track_history[tid] = []
            self.track_history[tid].append((x, y, w, h))
            if len(self.track_history[tid]) > 30:
                self.track_history[tid].pop(0)

            # predict
            history = self.track_history[tid]
            pred = self.predictor.predict(history)

            # velocity
            vx, vy = 0.0, 0.0
            if len(history) >= 2:
                vx = history[-1][0] - history[-2][0]
                vy = history[-1][1] - history[-2][1]

            # classify
            speed = (vx**2 + vy**2) ** 0.5
            cls = self.classifier.classify((x, y, w, h), (vx, vy), altitude=10)

            # tracked object msg
            obj = TrackedObject()
            obj.header = msg.header
            obj.track_id = tid
            obj.pose.position.x = x + w/2
            obj.pose.position.y = y + h/2
            obj.pose.orientation.w = 1.0
            obj.velocity.x = vx
            obj.velocity.y = vy
            obj.predicted_position.x = pred[0]
            obj.predicted_position.y = pred[1]
            obj.classification = cls
            obj.confidence = 1.0
            tracked_array.objects.append(obj)

            # sphere marker for track position
            m = Marker()
            m.header = msg.header
            m.ns = "tracks"
            m.id = tid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = (x + w/2) / 100.0
            m.pose.position.y = (y + h/2) / 100.0
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
            m.lifetime = rospy.Duration(1.0)
            marker_array.markers.append(m)

            # arrow marker for prediction
            a = Marker()
            a.header = msg.header
            a.ns = "predictions"
            a.id = tid + 1000
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.points.append(Point((x+w/2)/100.0, (y+h/2)/100.0, 0.0))
            a.points.append(Point(pred[0]/100.0, pred[1]/100.0, 0.0))
            a.scale.x = 0.05; a.scale.y = 0.1; a.scale.z = 0.1
            a.color.r = 1.0; a.color.g = 0.5; a.color.b = 0.0; a.color.a = 1.0
            a.lifetime = rospy.Duration(1.0)
            marker_array.markers.append(a)

        self.tracks_pub.publish(tracked_array)
        self.marker_pub.publish(marker_array)

        if len(tracks) > 0:
            rospy.loginfo(f"Tracking {len(tracks)} objects")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = TrackerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
