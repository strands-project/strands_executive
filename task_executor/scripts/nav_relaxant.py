#!/usr/bin/env python

import rospy
from strands_navigation_msgs.msg import TopologicalMap, NavStatistics, NavRoute
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks
from strands_executive_msgs import task_utils

class NavRelaxant(object):
    def __init__(self, count_threshold):
        super(NavRelaxant, self).__init__()     
        rospy.Subscriber('topological_map', TopologicalMap, self.map_callback)
        self.msg_store = MessageStoreProxy(collection='nav_stats')
        self.node_pairs = []
        self._allowed_to_turn_relax_on = True
        self._allowed_to_turn_relax_off = True
        self._count_threshold =  count_threshold


    def map_callback(self, msg):            
        node_pairs = []
        for node in msg.nodes:
            for edge in node.edges:
                node_pairs.append((node.name, edge.node, edge.edge_id))                
        self.node_pairs = node_pairs

    def pair_ok(self, start, end):
        """
            Report if we are happy with the stats between these two nodes
        """
        count = len(self.msg_store.query(NavStatistics._type, {"origin": start, "target": end, "final_node": end}))
        rospy.logdebug('nav stat check %s %s %s' % (start, end, count))
        return count > self._count_threshold


    def nav_stats_require_relaxed_operation(self):

        # only really needed for testing
        while len(self.node_pairs) == 0 and not rospy.is_shutdown():
            rospy.sleep(1)
            rospy.loginfo('Waiting for nodes')


        for (start, end, edge_id) in self.node_pairs:
            if not self.pair_ok(start, end):
                rospy.loginfo('Nav stats from %s to %s not sufficient to stop being relaxed' % (start, end))
                return True, (start, end, edge_id)
        return False, None


    def start(self):
        Thread(target=self.run).start()

    def generate_edge_task(self, start, end, edge_id, window_size):
        rospy.loginfo('Generating edge task %s to %s via %s' % (start, end, edge_id))

        # get task services
        add_task_srv_name = '/task_executor/add_tasks'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_task_srv_name)
        rospy.loginfo("Done")        
        add_tasks_srv = rospy.ServiceProxy(add_task_srv_name, AddTasks)
        
        # create a task to use the edge in question
        task = Task()
        task.action = '/topological_navigation/execute_policy_mode'
        task.max_duration = rospy.Duration(300)
        task.start_after = rospy.get_rostime()
        task.end_before = task.start_after + rospy.Duration(window_size)
        task.start_node_id = start
        task.end_node_id = end

        route = NavRoute()
        route.source = [start]
        route.edge_id = [edge_id]

        msg_store = MessageStoreProxy()
        task_utils.add_object_id_argument(task, msg_store.insert(route), NavRoute)

        # add the task
        add_tasks_srv([task])


    def run(self):
        # rate = rospy.Rate(0.5) 

        repeat_window_secs = 60.0 * 20.0
        rate = rospy.Rate(1.0/repeat_window_secs) 

        while not rospy.is_shutdown():

            try:

                relax_is_on = rospy.get_param('relaxed_nav', False)         
                rospy.loginfo('relax_is_on %s' % relax_is_on)

                if (relax_is_on and self._allowed_to_turn_relax_off) or (not relax_is_on and self._allowed_to_turn_relax_on):
                    relax_should_be, edge_tuple = self.nav_stats_require_relaxed_operation()    
                    
                    if relax_is_on != relax_should_be:
                        rospy.set_param('relaxed_nav', relax_should_be)
                        rospy.loginfo('Transitioning relaxed_nav parameter to: %s' % relax_should_be)       
                    
                    if relax_should_be and rospy.get_param('explore_edges', True):
                        self.generate_edge_task(*edge_tuple, window_size=repeat_window_secs)

            except Exception, e:
                rospy.logwarn('while checking relaxation state: %s' % e)

            finally:
                rate.sleep()

if __name__ == '__main__':
    rospy.init_node('nav_relaxant')
    relaxant = NavRelaxant(count_threshold=rospy.get_param('~count_threshold', 10))
    relaxant.run()
    rospy.spin()
