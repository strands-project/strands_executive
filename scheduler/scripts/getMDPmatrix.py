#!/usr/bin/env python

import sys
import rospy
import threading
from strands_executive_msgs.srv import GetExpectedTravelTimesToWaypoint

expected_time_lock = threading.RLock()


def expected_time(start, end, kind):
  if kind == "top":
    from strands_navigation_msgs.srv import EstimateTravelTime
    expected_time_srv_name = 'topological_navigation/travel_time_estimator'
    rospy.loginfo('Waiting for %s' % expected_time_srv_name)
    rospy.wait_for_service(expected_time_srv_name)
    rospy.loginfo('... and got %s' % expected_time_srv_name)
    expected_time_srv = rospy.ServiceProxy(expected_time_srv_name, EstimateTravelTime) 
    return expected_time_srv(start=start, target=end).travel_time       
  elif kind == "mdp":
    from strands_executive_msgs.srv import GetExpectedTravelTimesToWaypoint
    expected_time_srv_name = 'mdp_plan_exec/get_expected_travel_times_to_waypoint'
    rospy.loginfo('Waiting for %s' % expected_time_srv_name)
    rospy.wait_for_service(expected_time_srv_name)
    rospy.loginfo('... and got %s' % expected_time_srv_name)
    expected_time_srv = rospy.ServiceProxy(expected_time_srv_name, GetExpectedTravelTimesToWaypoint)
    epoch = rospy.get_rostime()      
    try:            
      # expected_time_lock is reentrant
      expected_time_lock.acquire()
      resp = expected_time_srv(target_waypoint=target, epoch=epoch)
    finally:
      expected_time_lock.release()
    return resp.travel_times[resp.source_waypoints.index(start)]
        

def get_navigation_duration(start, end, kind):

  try:            
    # prevent concurrent calls to expected_time service. 
    expected_time_lock.acquire()
    if start == '' or end == '':
      # if we're going nowhere, return some default
      return rospy.Duration(10)
    else:
      et = expected_time(start, end,kind)
      return rospy.Duration(max(et.to_sec(), 10)) #TODO what is exactly this conversion doing?
  except Exception, e:
    rospy.logwarn('Caught exception when getting expected time: %s' % e)
    return rospy.Duration(10)
  finally:
    expected_time_lock.release()

if __name__ == "__main__":
    
  names = []
  f = file("/home/lenka/phd/text/Articles/ECMR/Data/Matrix/top_labels.txt","w")
  names.append("ChargingPoint")
  f.write(" 0;")

  for i in range(46):
    names.append("WayPoint"+str(i))
    if(i<9):
      f.write(" "+str(i)+";")
    else:
      f.write(str(i)+";")

  f.close()

  f = file("/home/lenka/phd/text/Articles/ECMR/Data/Matrix/top.txt","w")

  #TODO what happend if I request time between two waypoints where is no edge?
  for i in range(47):
    for j in range(47):
      dur= get_navigation_duration(names[i],names[j],"top")
      f.write(str(dur.to_sec())+";")
    f.write("\n")

  f.close()
  

