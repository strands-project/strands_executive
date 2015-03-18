#!/usr/bin/env python

import rospy

from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from std_msgs.msg import String
from random import shuffle
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import TopologicalMap


class PatrolScheduler(object):
    """ Repeatedly sends a randomised list of task to the task executor. These don't have tasks associated with them. If you want to add tasks, see fifo_tester.py for an example."""
    def __init__(self):
        self.node_names = []
        rospy.Subscriber('topological_map', TopologicalMap, self.map_callback)

        # load waypoint from datacentre
        self.waypoints = self.get_nodes()
        rospy.loginfo('Patrolling the following nodes: %s' % self.waypoints)
        self.current_waypoint = ''
        self.add_tasks_srv = None
        rospy.Subscriber("/current_node", String, self.current_node_cb)
        
        add_tasks_srv_name = 'task_executor/add_tasks'
        set_exe_stat_srv_name = 'task_executor/set_execution_status'
        rospy.loginfo("Waiting for task_executor service...") 
        rospy.wait_for_service(add_tasks_srv_name) 
        rospy.wait_for_service(set_exe_stat_srv_name) 
        rospy.loginfo("Done") 
        self.add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks) 
        set_execution_status_srv = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        
        shuffle(self.waypoints) 
        self.send_tasks() 

        try: 
            # Make sure the task executor is running 
            set_execution_status_srv(True) 
        except rospy.ServiceException, e: 
            print "Service call failed: %s"%e


    def map_callback(self, msg):        
        print 'got map: %s' % len(msg.nodes)
        self.node_names = [node.name for node in msg.nodes]
        
    def get_nodes(self):
        while len(self.node_names) == 0:
            print 'no nodes'
            rospy.sleep(1)
        return self.node_names

    def send_tasks(self): 
        rospy.loginfo("Sending next batch of patrol tasks")

        patrol_tasks = [Task(start_node_id=wp, end_node_id=wp) for wp in self.waypoints]

        self.add_tasks_srv(patrol_tasks)
    

    def current_node_cb(self, data):
    
        # if we have just changed nodes 
        if self.current_waypoint != data.data: 
            rospy.loginfo("Changed nodes to %s from %s" % (data.data, self.current_waypoint))
            self.current_waypoint = data.data
            # if we're on the last waypoint, send the tasks again
            # actually this is incorrect as we could pass through a node on the way to somewhere else, but it doesn't really matter
            if self.current_waypoint == self.waypoints[-1]:
                shuffle(self.waypoints) 
                self.send_tasks()
        


if __name__ == '__main__':
    rospy.init_node("patrol_scheduler")
    
    patrol_scheduler = PatrolScheduler()
    rospy.spin()




