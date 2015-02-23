#!/usr/bin/env python


from urllib import urlretrieve
from tempfile import mkstemp
from icalendar import Calendar, Event, vDatetime
from strands_executive_msgs.msg import Task
from os import remove
from std_msgs.msg import Time
from pprint import pprint
import rospy
from datetime import datetime
from time import mktime


class ICalTools:

    def from_url(self, url):
        (_, f) = mkstemp()

        try:
            urlretrieve(url, f)
            g = open(f, 'rb')
            self.gcal = Calendar.from_ical(g.read())
        finally:
            try:
                remove(f)
            except:
                rospy.logwarn('could not delete temp file %s', f)

    def to_task(self):
        t = Task()
        tasks = []
        for component in self.gcal.walk():
            if component.name == "VEVENT":
                start = mktime(component.decoded('dtstart').timetuple())
                end = mktime(component.decoded('dtend').timetuple())
                t.start_after = rospy.Time.from_sec(start)
                t.end_before = rospy.Time.from_sec(end)
                t.max_duration = (t.end_before - t.start_after) / 2
                t.action = component.get('summary')
                t.start_node_id = component.get('description')
                tasks.append(t)
        return tasks

if __name__ == '__main__':
    ical = ICalTools()
    ical.from_url('https://www.google.com/calendar/ical/hanheide.net_c8l2kpajdshp8p1bc3u6r1qsjk%40group.calendar.google.com/private-176de9a6b071a9c0f4c84d06b3dc7b2d/basic.ics')
    print ical.to_task()
