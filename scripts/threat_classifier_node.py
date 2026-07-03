#!/usr/bin/env python3
import rospy
import sys
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from threat_classifier import ThreatClassifier
from drone_delivery.msg import TrackedObjectArray, ThreatClassArray, ThreatClass

class ThreatClassifierNode:
    def __init__(self):
        rospy.init_node("threat_classifier_node", anonymous=False)
        self.classifier = ThreatClassifier()

        rospy.Subscriber("/tracked_objects", TrackedObjectArray, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/threat_classes", ThreatClassArray, queue_size=10)
        rospy.loginfo("Threat classifier node ready.")

    def callback(self, msg):
        class_array = ThreatClassArray()
        class_array.header = msg.header

        for obj in msg.objects:
            vx = obj.velocity.x
            vy = obj.velocity.y
            w = 50.0
            h = 50.0
            cls = self.classifier.classify(
                (obj.pose.position.x, obj.pose.position.y, w, h),
                (vx, vy),
                altitude=10.0
            )
            motion_model = "CV" if cls == "bird" else "IMM"

            tc = ThreatClass()
            tc.track_id = obj.track_id
            tc.classification = cls
            tc.motion_model = motion_model
            tc.confidence = obj.confidence
            class_array.classes.append(tc)

        self.pub.publish(class_array)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = ThreatClassifierNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
