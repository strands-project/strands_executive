#! /usr/bin/env python

import sys
import os
import rospy

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker

from strands_executive_msgs.srv import GetExpectedTravelTimesToWaypoint, GetExpectedTravelTimesToWaypointResponse, GetSpecialWaypoints


class MdpTravelTimeEstimator(object):

    def __init__(self,top_map):
        
        self.top_map_mdp=TopMapMdp(top_map)
        self.directory = os.path.expanduser("~") + '/tmp/prism/time_estimator/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name=top_map+".mdp"
        self.travel_times_to_waypoint_service = rospy.Service('/mdp_plan_exec/get_expected_travel_times_to_waypoint', GetExpectedTravelTimesToWaypoint, self.travel_times_to_waypoint_cb)                
        self.prism_estimator=PrismJavaTalker(8085,self.directory, self.file_name)        
        self.last_epoch=-1
        rospy.loginfo("MDP travel times estimator initialised.")

    def travel_times_to_waypoint_cb(self,req):
        print "ENTERD SERVICE"
        if req.epoch != self.last_epoch:
            self.last_epoch=req.epoch
            self.top_map_mdp.set_mdp_action_durations(self.directory+self.file_name, req.epoch)            
        specification='R{"time"}min=? [ ( F "' + req.target_waypoint + '") ]'
        state_vector=map(rospy.Duration, self.prism_estimator.get_state_vector(specification))
        state_vector_names=self.top_map_mdp.parse_sta_to_waypoints(self.directory+'original.sta', len(state_vector))
        print "LEFT SERVICE"
        return GetExpectedTravelTimesToWaypointResponse(source_waypoints=state_vector_names, travel_times=state_vector)
        

    def main(self):
       # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_estimator.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_travel_time_estimator')
    filtered_argv=rospy.myargv(argv=sys.argv)        
    if len(filtered_argv)<2:
        rospy.logerr("No topological map provided. usage: rosrun mdp_plan_exec mdp_travel_time_estimator <topological_map_name>")
        sys.exit(2)
    elif len(filtered_argv)>2:
        rospy.logwarn("Too many arguments. Assuming topological map is the first one. usage: rosrun mdp_plan_exec mdp_travel_time_estimator <topological_map_name>")
    
    mdp_estimator =  MdpTravelTimeEstimator(filtered_argv[1])
    mdp_estimator.main()
    
    
