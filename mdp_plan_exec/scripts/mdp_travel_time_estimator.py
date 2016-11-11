#! /usr/bin/env python

import sys
import os
import rospy

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker

from strands_executive_msgs.srv import GetExpectedTravelTimesToWaypoint, GetExpectedTravelTimesToWaypointResponse, GetSpecialWaypoints


class MdpTravelTimeEstimator(object):

    def __init__(self):
        
        self.top_map_mdp=TopMapMdp()
        self.directory = os.path.expanduser("~") + '/tmp/prism/time_estimator/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name="topo_map.mdp"
        self.prism_estimator=PrismJavaTalker(8085,self.directory, self.file_name)        
        self.last_epoch=-1
        self.travel_times_to_waypoint_service = rospy.Service('/mdp_plan_exec/get_expected_travel_times_to_waypoint', GetExpectedTravelTimesToWaypoint, self.travel_times_to_waypoint_cb)                
        rospy.loginfo("MDP travel times estimator initialised.")

    def travel_times_to_waypoint_cb(self,req):
        if not self.top_map_mdp.target_in_topological_map(req.target_waypoint):
            rospy.logerr("Get travel times target  " + req.target_waypoint  + "  is not a node in the topological map. Returning empty response")
            return GetExpectedTravelTimesToWaypointResponse()
        if req.epoch != self.last_epoch:
            self.last_epoch=req.epoch
            self.top_map_mdp.add_predictions(self.directory+self.file_name, req.epoch)           
        specification='R{"time"}min=? [ ( F "' + req.target_waypoint + '") ]'
        rospy.loginfo("The specification is " + specification)
        self.top_map_mdp.create_top_map_mdp_structure()
        state_vector=map(rospy.Duration, self.prism_estimator.get_state_vector(specification))
        state_vector_names=self.top_map_mdp.parse_sta_to_waypoints(self.directory+'original.sta', len(state_vector))
        return GetExpectedTravelTimesToWaypointResponse(source_waypoints=state_vector_names, travel_times=state_vector)
        

    def main(self):
       # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_estimator.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_travel_time_estimator')

    mdp_estimator =  MdpTravelTimeEstimator()
    mdp_estimator.main()
    
    
