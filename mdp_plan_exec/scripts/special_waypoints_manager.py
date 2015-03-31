#! /usr/bin/env python
import rospy
from strands_executive_msgs.srv import AddDeleteSpecialWaypoint, AddDeleteSpecialWaypointRequest, GetSpecialWaypoints, GetSpecialWaypointsResponse

   
class SpecialWaypointsManager(object):

    def __init__(self):    
        self.forbidden_waypoints=[]
        self.forbidden_waypoints_ltl_string=''
        
        self.safe_waypoints=[]
        self.safe_waypoints_ltl_string=''
        
        self.add_delete_special_waypoint_srv = rospy.Service('/mdp_plan_exec/add_delete_special_waypoint', AddDeleteSpecialWaypoint, self.add_delete_special_waypoint_cb)
        self.get_special_waypoints_srv=rospy.Service('/mdp_plan_exec/get_special_waypoints', GetSpecialWaypoints, self.get_special_waypoints_cb)
                
    def add_delete_special_waypoint_cb(self,req):
        if req.waypoint_type == AddDeleteSpecialWaypointRequest.FORBIDDEN:
            if req.is_addition:
                self.add_forbidden_waypoint(req.waypoint)
            else:
                self.del_forbidden_waypoint(req.waypoint)               
        if req.waypoint_type == AddDeleteSpecialWaypointRequest.SAFE:
            if req.is_addition:
                self.add_safe_waypoint(req.waypoint)
            else:
                self.del_safe_waypoint(req.waypoint)               
        return True
    
    def add_forbidden_waypoint(self,waypoint):
        if waypoint in self.forbidden_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' already in forbidden waypoint list.')
        else:
            self.forbidden_waypoints.append(waypoint)
            self.set_forbidden_waypoints_ltl_string()
                   
    def del_forbidden_waypoint(self,waypoint):
        if waypoint not in self.forbidden_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' not in forbidden waypoint list.')
        else:
            del self.forbidden_waypoints[self.forbidden_waypoints.index(waypoint)]
            self.set_forbidden_waypoints_ltl_string()        
        
    def add_safe_waypoint(self,waypoint):
        if waypoint in self.safe_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' already in safe waypoint list.')
        else:
            self.safe_waypoints.append(waypoint)
            self.set_safe_waypoints_ltl_string()
    
    def del_safe_waypoint(self,waypoint):
        if waypoint not in self.safe_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' not in safe waypoint list.')
        else:
            del self.safe_waypoints[self.safe_waypoints.index(waypoint)]
            self.set_safe_waypoints_ltl_string()    
        
    def set_forbidden_waypoints_ltl_string(self):
        self.forbidden_waypoints_ltl_string=''       
        for i in range(0,len(self.forbidden_waypoints)):
            self.forbidden_waypoints_ltl_string=self.forbidden_waypoints_ltl_string + '"' + self.forbidden_waypoints[i] + '" & !'        
        if not self.forbidden_waypoints_ltl_string=='':
            self.forbidden_waypoints_ltl_string='(!' + self.forbidden_waypoints_ltl_string[:-4] + ')'
                    
    def set_safe_waypoints_ltl_string(self):
        self.safe_waypoints_ltl_string=''        
        for i in range(0,len(self.safe_waypoints)):
            self.safe_waypoints_ltl_string=self.safe_waypoints_ltl_string  + '"' + self.safe_waypoints[i] + '"'  + ' | '        
        if not self.safe_waypoints_ltl_string=='':
            self.safe_waypoints_ltl_string='(' + self.safe_waypoints_ltl_string[:-3] + ')'
             
    def get_special_waypoints_cb(self, req):
        return GetSpecialWaypointsResponse(safe_waypoints=self.safe_waypoints,
                                        safe_waypoints_ltl_string=self.safe_waypoints_ltl_string,
                                        forbidden_waypoints=self.forbidden_waypoints,
                                        forbidden_waypoints_ltl_string=self.forbidden_waypoints_ltl_string)
        
    def main(self):
        # Wait for control-c
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('special_waypoint_manager')
    manager =  SpecialWaypointsManager()
    manager.main()
    