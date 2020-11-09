#!/usr/bin/env python
from nav_msgs.srv import GetMap,GetMapResponse
import rospy

def Get_Map_Callback(self):
    print(GetMap.__slots__)
    return {'map': GetMap()}


rospy.init_node('Get_map_server')
s = rospy.Service('static_map', GetMap,  Get_Map_Callback)

print("Ready to get map.")
rospy.spin()

