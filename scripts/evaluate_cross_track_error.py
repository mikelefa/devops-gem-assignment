#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import sys
import time

class CrossTrackEvaluator:
    def __init__(self):
        self.max_cte = 0.0
        self.start_time = time.time()
        self.timeout_sec = 60  # Total simulation timeout
        self.msg_check_window = 10  # Initial seconds to wait for first message
        self.last_msg_time = None

        rospy.init_node('cte_evaluator', anonymous=True)
        rospy.Subscriber('/cross_track_error', Float32, self.cte_callback)
        rospy.loginfo("Evaluator started. Waiting for /cross_track_error messages...")

    def cte_callback(self, msg):
        cte = abs(msg.data)
        self.last_msg_time = time.time()
        if cte > self.max_cte:
            self.max_cte = cte
        rospy.loginfo(f"Current CTE: {cte:.3f} m | Max CTE so far: {self.max_cte:.3f} m")

    def wait_for_first_message(self):
        """Wait a few seconds for the first message to show simulation is running."""
        rospy.loginfo(f"Waiting up to {self.msg_check_window} seconds for first message...")
        start_wait = time.time()
        rate = rospy.Rate(2)  # Check twice per second
        while time.time() - start_wait < self.msg_check_window:
            if self.last_msg_time is not None:
                rospy.loginfo("First message received.")
                return True
            rate.sleep()
        rospy.logerr("No messages received on /cross_track_error. Is the simulation running?")
        return False

    def run(self):
        if not self.wait_for_first_message():
            sys.exit(1)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            elapsed = time.time() - self.start_time
            if elapsed > self.timeout_sec:
                rospy.loginfo(f"Evaluation timeout after {self.timeout_sec} seconds.")
                break
            rate.sleep()

        rospy.loginfo(f"Final Max Cross Track Error: {self.max_cte:.3f} m")

        if self.max_cte < 1.0:
            rospy.loginfo("Test PASSED: Max CTE < 1 meter")
            sys.exit(0)
        else:
            rospy.logerr("Test FAILED: Max CTE >= 1 meter")
            sys.exit(1)

if __name__ == '__main__':
    try:
        evaluator = CrossTrackEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        pass
