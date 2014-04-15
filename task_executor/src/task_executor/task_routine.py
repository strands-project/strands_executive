import rospy
from datetime import *

epoch = datetime.utcfromtimestamp(0)

def unix_time(dt):
    """
    Converts a datetime object to seconds since epoch
    """    
    delta = dt - epoch
    return delta.total_seconds()

def time_to_secs(time):
	return (time.hour * 60 * 60) + (time.minute * 60) + (time.second) + (time.microsecond/1000.0)

    # start by providing bounds on daily exectuion

    # given a task or list of tasks (which could be produce by that task at all waypoints)

    # do it n times in a specific window,  - scheduled for proposal x secs before the window

    # do it n times every X duration 
