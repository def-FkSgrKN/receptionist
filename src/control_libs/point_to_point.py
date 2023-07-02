#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import numpy as np
import time

# 1
# X = 0.9511580467224121
# Y = -4.140828609466553
# YAW = np.pi + np.pi / 2

"""
GOAL = [
    [-2.259922504425049, -0.09336382150650024, 0],
    [0.9511580467224121, -4.140828609466553, np.pi + np.pi / 2],
    [4.437300682067871, 0.43928956985473633, 0]
]
"""

#四捨五入
#目標1 [1.3657, 0.0328, np.pi/2],
#目標2 [0.8410, 0.0328, np.pi]

GOAL = [
    [1.3657, 0.0328, np.pi/2],
    [0.8410, 0.0328, np.pi]
]

# 2
# X = 4.437300682067871
# Y = 0.43928956985473633
# YAW = 0

# 0
# X = -2.259922504425049
# Y = -0.09336382150650024
# YAW = 0

class PointToPoint():
    def __init__(self):
        #rospy.init_node("point_to_point_node", anonymous=True)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback)
        self.status = 0

    def send_goal(self, goal):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = np.sin(goal[2] / 2)
        msg.pose.orientation.w = np.cos(goal[2] / 2)

        self.goal_pub.publish(msg)

    def callback(self, msg):
        status = GoalStatus()

        if msg.status_list != None:
            if len(msg.status_list) != 0:
                status = msg.status_list[0]
                self.status = status.status

def main_test():
    p = PointToPoint()
    time.sleep(2)

    p.send_goal(GOAL[0])

    while True:
        rospy.loginfo("1")
        if p.status == 3:
            break

    p.send_goal(GOAL[1])

    while True:
        rospy.loginfo("2")
        if p.status == 3:
            break

    p.send_goal(GOAL[2])
    
    while True:
        rospy.loginfo("3")
        if p.status == 3:
            break

    # rospy.spin()


if __name__ == "__main__":
    main_test()
    
