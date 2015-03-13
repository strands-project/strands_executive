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

    def __init__(self):
        self.tz_utc = tz.gettz('UTC')

    def from_url(self, url):
        response = urllib.urlopen(url)
        self.gcal = json.loads(response.read())

    def from_file(self, fname):
        g = open(fname, 'rb')
        self.gcal = json.loads(g.read())
        g.close()

    def task_from_gcal(self, gcal_event):

        start = parser.parse(gcal_event['start']['dateTime'])
        start_utc = start.astimezone(self.tz_utc)
        end = parser.parse(gcal_event['end']['dateTime'])
        end_utc = end.astimezone(self.tz_utc)

        t = Task()
        t.start_after = rospy.Time.from_sec(mktime(start_utc.timetuple()))
        t.end_before = rospy.Time.from_sec(mktime(end_utc.timetuple()))
        if 'location' in gcal_event:
            t.start_node_id = gcal_event['location']
            t.end_node_id = gcal_event['location']
        t.max_duration = (t.end_before - t.start_after) / 2
        t.action = gcal_event['summary']
        if 'description' in gcal_event:
            extra_args = load(gcal_event['description'])
        return t

    def to_task_list(self):
        tasks = []
        for gcal_event in self.gcal['items']:
            try:
                tasks.append(self.task_from_gcal(gcal_event))
            except Exception, e:
                rospy.logerr('failed to convert event from iCal to task: %s', str(e))
        return tasks

if __name__ == '__main__':
    rospy.init_node('ical_test')
    ical = ICalTools()
    #ical.from_url('https://www.googleapis.com/calendar/v3/calendars/henry.strands%40hanheide.net/events?key=AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0&singleEvents=true&orderBy=startTime&maxResults=2500')
    ical.from_file("test.json")
    pprint(ical.to_task_list())
