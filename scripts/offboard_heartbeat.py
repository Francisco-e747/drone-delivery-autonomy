#!/usr/bin/env python3
import rospy
import time
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardHeartbeat:
    def __init__(self):
        rospy.init_node("offboard_heartbeat", anonymous=False)
        self.current_state = State()
        self.rate = rospy.Rate(20)

        self.att_pub = rospy.Publisher(
            "/mavros/setpoint_raw/attitude",
            AttitudeTarget, queue_size=10)

        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.loginfo("Offboard heartbeat ready.")

    def state_callback(self, msg):
        self.current_state = msg

    def publish_attitude(self):
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.type_mask = 7
        msg.orientation.w = 1.0
        msg.thrust = 0.5
        self.att_pub.publish(msg)

    def run(self):
        rospy.loginfo("Publishing attitude setpoints for 10 seconds...")
        t0 = time.time()
        while time.time() - t0 < 10.0:
            self.publish_attitude()
            self.rate.sleep()

        rospy.loginfo("Switching to OFFBOARD mode...")
        for attempt in range(20):
            self.publish_attitude()
            resp = self.set_mode_client(custom_mode="OFFBOARD")
            if resp.mode_sent:
                rospy.loginfo("OFFBOARD mode set.")
                break
            rospy.logwarn(f"OFFBOARD attempt {attempt+1} failed")
            time.sleep(0.3)

        rospy.loginfo("Arming...")
        for attempt in range(20):
            self.publish_attitude()
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo("Armed!")
                break
            rospy.logwarn(f"Arm attempt {attempt+1} failed")
            time.sleep(0.3)

        rospy.loginfo("Maintaining OFFBOARD for 60 seconds...")
        start = rospy.Time.now()
        drops = 0
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > 60.0:
                rospy.loginfo(f"Done. Mode drops: {drops}")
                break
            if self.current_state.mode != "OFFBOARD":
                drops += 1
                self.set_mode_client(custom_mode="OFFBOARD")
            self.publish_attitude()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = OffboardHeartbeat()
        node.run()
    except rospy.ROSInterruptException:
        pass
