#!/usr/bin/env python
# jetracer_ros/scripts/collision_safety_node.py
# TODO: Implement safety logic (command timeouts, obstacle detection stops, emergency stop).

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CollisionSafetyNode:
    def __init__(self):
        rospy.init_node('collision_safety_node', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('cmd_vel_safety', Twist, queue_size=10)
        self.estop_sub = rospy.Subscriber('safety/emergency_stop', Bool, self.estop_callback)
        self.scan_sub = None # TODO: Subscribe to /scan or /range/front if sensor exists
        
        self.estop_active = False
        self.timeout = rospy.get_param('~timeout', 0.5)
        
        rospy.loginfo("Collision Safety Node initialized (Proposed Skeleton).")

    def estop_callback(self, msg):
        self.estop_active = msg.data
        if self.estop_active:
            rospy.logwarn("EMERGENCY STOP ACTIVATED!")
            self.stop_robot()

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.estop_active:
                self.stop_robot()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = CollisionSafetyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
