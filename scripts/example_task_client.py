#!/usr/bin/env python

import rospy
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair
import ros_datacentre.util as dc_util
from strands_executive_msgs.msg import Task
from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO
from strands_executive_msgs import task_utils
# import strands_executive_msgs


if __name__ == '__main__':
    rospy.init_node("example_task_client")

    # need message store to pass objects around
    msg_store = MessageStoreProxy() 

    # get the pose of a named object
    pose_name = "my favourite pose"

    try:
        # get the pose if it's there
        message, meta =  msg_store.query_named(pose_name, Pose._type)
        # if it's not there, add it in
        if message == None: 
        	message = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
        	pose_id = msg_store.insert_named(pose_name, message)
        else:
        	pose_id = meta["_id"]	        

        task = Task(node_id='WayPoint1')        
        task_utils.add_string_argument(task, 'hello world')
        task_utils.add_object_id_argument(task, pose_id, Pose)
        print task
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        


