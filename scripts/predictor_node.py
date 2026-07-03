#!/usr/bin/env python3
import rospy
import sys
sys.path.insert(0, '/home/javier/catkin_ws/src/drone_delivery/context1')
from predictor import ConstantVelocityPredictor
from drone_delivery.msg import TrackedObjectArray, ThreatArray, Threat
from geometry_msgs.msg import Point

class PredictorNode:
    def __init__(self):
        rospy.init_node("predictor_node", anonymous=False)
        self.predictor = ConstantVelocityPredictor(dt=0.5)
        self.track_history = {}

        rospy.Subscriber("/tracked_objects", TrackedObjectArray, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/predicted_threats", ThreatArray, queue_size=10)
        rospy.loginfo("Predictor node ready.")

    def callback(self, msg):
        threat_array = ThreatArray()
        threat_array.header = msg.header

        for obj in msg.objects:
            tid = obj.track_id
            x = obj.pose.position.x
            y = obj.pose.position.y

            if tid not in self.track_history:
                self.track_history[tid] = []
            self.track_history[tid].append((x, y, 50, 50))
            if len(self.track_history[tid]) > 30:
                self.track_history[tid].pop(0)

            history = self.track_history[tid]
            pred = self.predictor.predict(history)

            vx = obj.velocity.x
            vy = obj.velocity.y
            speed = (vx**2 + vy**2) ** 0.5
            uncertainty = max(0.5, speed * 0.5 * 2)

            threat = Threat()
            threat.header = msg.header
            threat.track_id = tid
            threat.position = obj.pose.position
            threat.predicted_position = Point(pred[0], pred[1], 0.0)
            threat.velocity = obj.velocity
            threat.uncertainty_radius = uncertainty
            threat.classification = obj.classification
            threat_array.threats.append(threat)

        self.pub.publish(threat_array)
        if len(threat_array.threats) > 0:
            rospy.loginfo(f"Published {len(threat_array.threats)} threats")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = PredictorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
