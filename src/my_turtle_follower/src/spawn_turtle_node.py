#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn

rospy.init_node('spawn_turtle_node')

rospy.wait_for_service('/spawn')
spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)

spawn_turtle(5, 5, 0, 'turtle2')

rospy.spin()
