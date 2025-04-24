#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import sys
import time

class CrossTrackEvaluator:
    def __init__(self):
        self.max_cte = 0.0
        self.start_time = time.time()
        self.timeout_sec = 60  # Simulation timeout

        rospy.init_node('cte_evaluator', anonymous=True)
        rospy.Subscriber('/cross_track_error', Float32, self.cte_callback)
        rospy.loginfo("Evaluator started. Listening for /cross_track_error...")

    def cte_callback(self, msg):
        cte = abs(msg.data)
        if cte > self.max_cte:
            self.max_cte = cte
        rospy.loginfo(f"Current CTE: {cte:.3f} m | Max CTE so far: {self.max_cte:.3f} m")

    def run(self):
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
