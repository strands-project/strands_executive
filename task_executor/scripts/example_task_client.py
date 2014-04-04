#!/usr/bin/env python

import rospy
import ros_datacentre_msgs.srv as dc_srv
from ros_datacentre_msgs.msg import StringPair
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask, SetExecutionStatus
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

        # pose_id = '533e9d6154a6f71c18fade50'

        task = Task(node_id='WayPoint1', action='test_task')        
        task_utils.add_string_argument(task, 'hello world')
        task_utils.add_object_id_argument(task, pose_id, Pose)

        print task

        # now register this with the executor
        add_task_srv_name = '/task_executor/add_task'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_task_srv_name)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Done")        
        
        add_task_srv = rospy.ServiceProxy(add_task_srv_name, AddTask)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        print add_task_srv(task)

        # Make sure the task executor is running
        set_execution_status(True)


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        


