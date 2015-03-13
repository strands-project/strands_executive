import rospy
from strands_executive_msgs.srv import  GetSpecialWaypoints

class SpecialWaypointsClient(object):
    def __init__(self):
        self.get_special_waypoints_srv=rospy.ServiceProxy('/mdp_plan_exec/get_special_waypoints',GetSpecialWaypoints)
                                        
    def get_safe_waypoints_ltl_string(self):
        return self.get_special_waypoints_srv().safe_waypoints_ltl_string
    
    def get_forbidden_waypoints_ltl_string(self):
        return self.get_special_waypoints_srv().forbidden_waypoints_ltl_string
