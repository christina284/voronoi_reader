#!/usr/bin/env python
import rospy
from tuw_multi_robot_msgs.msg import Graph



def callback(msg):

	# posx = msg.origin.position.x
	# posy = msg.origin.position.y
	# posz = msg.origin.position.z
	# quatx = msg.origin.orientation.x
	# quaty = msg.origin.orientation.y
	# quatz = msg.origin.orientation.z
	# quatw = msg.origin.orientation.w
	# v_id = msg.vertices.id
	# pathx = msg.vertices.path.x
	# pathy = msg.vertices.path.y
	# pathz = msg.vertices.path.z
	# weight = msg.vertices.weight
	# successors = msg.vertices.successors
	predecessors = msg.vertices.predecessors
    return predecessors


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)
    predecessors = rospy.Subscriber("chatter", Graph, callback)
	print(predecessors)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()