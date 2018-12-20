#!/usr/bin/env python

from strands_executive_msgs.srv import GetBlacklistedNodes, GetBlacklistedNodesResponse
import rospy

blacklisted_nodes = ['WayPoint3', 'WayPoint8']

def handle_get_blacklisted_nodes(req):
    print("Returning %s" % blacklisted_nodes)
    return GetBlacklistedNodesResponse(blacklisted_nodes)

def get_blacklisted_nodes_server():
    rospy.init_node('get_blacklisted_nodes_server')
    s = rospy.Service('task_executor/get_blacklisted_nodes', GetBlacklistedNodes, handle_get_blacklisted_nodes)
    rospy.spin()

if __name__ == "__main__":
    get_blacklisted_nodes_server()