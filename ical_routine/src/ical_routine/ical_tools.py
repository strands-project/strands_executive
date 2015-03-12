#!/usr/bin/env python


from tempfile import mkstemp
from strands_executive_msgs.msg import Task
from os import remove
from std_msgs.msg import Time
from pprint import pprint
import rospy
from datetime import datetime
from time import mktime
from yaml import load
import urllib
import json
from dateutil import parser
from dateutil import tz



class ICalTools:

    def from_url(self, url):
        response = urllib.urlopen(url)
        self.gcal = json.loads(response.read())

    def from_file(self, fname):
        g = open(fname, 'rb')
        self.gcal = json.loads(g.read())
        g.close()

    def to_task(self):
        tz_utc = tz.gettz('UTC')
        tasks = []
        #pprint(self.gcal)
        for component in self.gcal['items']:
            #pprint(component)
            try:
                start = parser.parse(component['start']['dateTime'])
                start_utc = start.astimezone(tz_utc)
                end = parser.parse(component['end']['dateTime'])
                end_utc = end.astimezone(tz_utc)

                t = Task()
                t.start_after = rospy.Time.from_sec(mktime(start_utc.timetuple()))
                t.end_before = rospy.Time.from_sec(mktime(end_utc.timetuple()))
                if 'location' in component:
                    t.start_node_id = component['location']
                    t.end_node_id = component['location']
                t.max_duration = (t.end_before - t.start_after) / 2
                t.action = component['summary']
                if 'description' in component:
                    t.arguments = load(component['description'])
                tasks.append(t)
            except Exception, e:
                rospy.logerr('failed to convert event from iCal to task: %s', str(e))
        return tasks

if __name__ == '__main__':
    rospy.init_node('ical_test')
    ical = ICalTools()
    #ical.from_url('https://www.googleapis.com/calendar/v3/calendars/henry.strands%40hanheide.net/events?key=AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0&singleEvents=true&orderBy=startTime&maxResults=2500')
    ical.from_file("test.json")
    pprint(ical.to_task())
