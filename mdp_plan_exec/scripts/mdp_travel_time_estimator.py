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

    def travel_times_to_waypoint_cb(self,req):
        #set state and use fremen to set costs and probs
        self.top_map_mdp.write_prism_model(self.directory+self.file_name)
        specification='R{"time"}min=? [ ( F "' + req.target_waypoint + '") ]'
        state_vector=self.prism_estimator.get_state_vector(specification)
        print state_vector       
        return GetExpectedTravelTimesToWaypointResponse(travel_times=state_vector)
        

    def main(self):
       # Wait for control-c
        rospy.spin()       
        if rospy.is_shutdown():
            self.prism_estimator.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_travel_time_estimator')
    
    if len(sys.argv)<2:
        print "usage: rosrun mdp_plan_exec mdp_travel_time_estimator <topological_map_name>"
        sys.exit(2)
        
    mdp_estimator =  MdpTravelTimeEstimator(sys.argv[1])
    mdp_estimator.main()
    
    
