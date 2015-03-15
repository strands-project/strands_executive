#!/usr/bin/env python
PKG = 'gcal_routine'


from strands_executive_msgs.msg import Task
import rospy
from time import mktime
from yaml import load, parse
import urllib
import json
from dateutil import parser
from dateutil import tz
from datetime import datetime
from roslib.packages import find_resource

from threading import Thread


def rostime_str(rt):
    return str(datetime.fromtimestamp(rt.secs))


class TaskConfigurator:

    def __init__(self):
        stream = open(rospy.get_param(
            '~/available_tasks_conf_file',
            find_resource(PKG, 'default_tasks.yaml')[0]),
            'r')
        self.available_tasks = load(stream)

    def _fill_slots(self, src, dest):
        for s in dest.__slots__:
            if s in src:
                # a bit of a hacky way to check if we need to recurse...
                if str(type(dest.__getattribute__(s))).startswith('<class ') \
                        or type(dest.__getattribute__(s)) == dict:
                    self._fill_slots(src[s], dest.__getattribute__(s))
                elif type(dest.__getattribute__(s)) == list:
                    dest.__getattribute__(s).extend(src[s])
                else:
                    dest.__setattr__(s, src[s])



    def fill_task(self, id, task):
        if id in self.available_tasks:
            rospy.loginfo('found task %s in available tasks', id)
            self._fill_slots(self.available_tasks[id], task)


class GCal:

    def __init__(self, calendar, key, add_cb=None,
                 remove_cb=None, update_wait=60, file_name=None):
        self.tz_utc = tz.gettz('UTC')
        if file_name is not None:
            self.uri = file_name
        else:
            self.uri = self._get_url(calendar, key)
        self.time_offset = rospy.Duration.from_sec(0)
        rospy.loginfo('using uri %s', self.uri)
        self.events = {}
        self.gcal = {}
        self.previous_events = {}
        self.update_wait = update_wait
        self.add_cb = add_cb
        self.remove_cb = remove_cb
        self.update_worker = Thread(target=self._update_run)

    def start_worker(self):
        self.update_worker.start()

    def _get_url(self, calendar, key, max_results=2500):
        return 'https://www.googleapis.com/calendar/v3/calendars/' \
               '%s/events?key=%s&singleEvents=true&' \
               'orderBy=startTime&maxResults=%d' % (calendar,
                                                    key, max_results)

    def _update_run(self):
        # make sure we can be killed here
        while not rospy.is_shutdown():
            added = []
            removed = []
            self.update(added, removed)
            # sleep until next check
            target = rospy.get_rostime()
            target.secs = target.secs + self.update_wait
            while rospy.get_rostime() < target and not rospy.is_shutdown():
                rospy.sleep(1)

    def shift_to_now(self):
        times = [s.start_after for s in self.events.values()]
        if len(times) < 1:
            return
        self.time_offset = min(times) - rospy.get_rostime()
        rospy.logdebug('now is %s', rostime_str(rospy.get_rostime()))
        for s in self.events.values():
            s.start_after = s.start_after - self.time_offset
            s.end_before = s.end_before - self.time_offset
            rospy.logdebug('new event times for %s: %s -> %s',
                           s.action,
                           rostime_str(s.start_after),
                           rostime_str(s.end_before))

    def update(self, added, removed):
        rospy.loginfo('updating from google calendar %s', self.uri)
        self.previous_events = self.events.copy()
        if self.uri.lower().startswith('http'):
            try:
                response = urllib.urlopen(self.uri)
                self.gcal = json.loads(response.read())
            except Exception, e:
                rospy.logerror('failed to get response from %s: %s',
                               self.uri, str(e))
                return
        else:
            g = open(self.uri, 'rb')
            self.gcal = json.loads(g.read())
            g.close()
        self._to_task_list()
        if self._find_changes(added, removed):
            rospy.loginfo('there were changes in the calendar to process +:%d -:%d',
                          len(added), len(removed))
            if self.add_cb is not None:
                for a in added:
                    self.add_cb(a, self.events[a])
            if self.remove_cb is not None:
                for r in removed:
                    self.remove_cb(r, self.previous_events[r])
            return True
        else:
            rospy.logdebug('no changes, keep watching')
            return False

    def get_task_list(self):
        return self.events

    def _find_changes(self, added=[], removed=[]):
        """
        identifies the change set. Returns True when a change has been found
        """
        new_ids = set(self.events.keys())
        prev_ids = set(self.previous_events.keys())
        additions = new_ids.difference(prev_ids)
        deletions = prev_ids.difference(new_ids)
        if len(additions) > 0 or len(deletions) > 0:
            added.extend(additions)
            removed.extend(deletions)
            return True
        else:
            return False

    def _task_from_gcal(self, gcal_event):
        start = parser.parse(gcal_event['start']['dateTime'])
        start_utc = start.astimezone(self.tz_utc)
        end = parser.parse(gcal_event['end']['dateTime'])
        end_utc = end.astimezone(self.tz_utc)

        t = Task()
        t.start_after = rospy.Time.from_sec(mktime(start_utc.timetuple())) \
            - self.time_offset
        t.end_before = rospy.Time.from_sec(mktime(end_utc.timetuple())) \
            - self.time_offset
        if 'location' in gcal_event:
            t.start_node_id = gcal_event['location']
            t.end_node_id = gcal_event['location']
        t.max_duration = (t.end_before - t.start_after) / 2
        t.action = gcal_event['summary']
        if 'description' in gcal_event:
            try:
                extra_args = load(gcal_event['description'])
            except Exception, e:
                rospy.logwarn("couldn't parse extra args %s: %s",
                              gcal_event['description'], str(e))
        return t

    def _to_task_list(self):
        self.events = {}
        for gcal_event in self.gcal['items']:
            try:
                k = gcal_event['id'] + gcal_event['updated']
                self.events[k] = self._task_from_gcal(gcal_event)
            except Exception, e:
                rospy.logerr('failed to convert event from iCal to task: %s', str(e))

if __name__ == '__main__':
    tc = TaskConfigurator()
    t = Task()
    tc.fill_task('wait', t)
    print t
