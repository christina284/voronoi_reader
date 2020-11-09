#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetMap
import sys
rospy.init_node('service_client')
rospy.wait_for_service('static_map')
try:
    Get_map = rospy.ServiceProxy('static_map', GetMap)
    resp1 = Get_map()
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)


