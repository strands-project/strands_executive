#!/usr/bin/env python
from strands_executive_msgs.msg import Task
import rospy
import json
import requests

from calendar import timegm
from dateutil import parser
from dateutil import tz
from datetime import datetime
from datetime import timedelta
from strands_executive_msgs.srv import CreateTask
from pprint import pprint

from threading import Thread

PKG = 'gcal_routine'

def rostime_str(rt):
    return str(datetime.fromtimestamp(rt.secs)) + '  ' + str(rt.secs)


class GCal:

    def __init__(self, calendar, key, add_cb=None,
                 remove_cb=None, update_wait=60, minTimeDelta=None,
                 maxTimeDelta=None, file_name=None, time_critical=False):
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
        self.minTimeDelta = minTimeDelta
        self.maxTimeDelta = maxTimeDelta
        self.time_critical = time_critical
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
        self.previous_events = self.events.copy()
        if self.uri.lower().startswith('http'):
            try:
                uri = self.uri
                now = datetime.now()
                if self.minTimeDelta is not None:
                    mt = now - timedelta(days=self.minTimeDelta)
                    uri = "%s&timeMin=%sZ" % (uri, mt.isoformat())
                if self.maxTimeDelta is not None:
                    mt = now + timedelta(days=self.maxTimeDelta)
                    uri = "%s&timeMax=%sZ" % (uri, mt.isoformat())
                rospy.loginfo('updating from google calendar %s', uri)
                response = requests.get(uri)
                self.gcal = json.loads(response.text)
            except Exception, e:
                rospy.logerr('failed to get response from %s: %s',
                             self.uri, str(e))
                return
        else:
            g = open(self.uri, 'rb')
            self.gcal = json.loads(g.read())
            g.close()
        self._to_task_list()
        if self._find_changes(added, removed):
            rospy.loginfo('changes in the calendar to process +%d -%d',
                          len(added), len(removed))

            for a in added:
                rospy.loginfo('instantiate %s' % a)
                self.events[a] = self.task_from_gcal(self.events[a])

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

    def task_from_gcal(self, gcal_event):
        start = parser.parse(gcal_event['start']['dateTime'])
        start_utc = start.astimezone(self.tz_utc)
        end = parser.parse(gcal_event['end']['dateTime'])
        end_utc = end.astimezone(self.tz_utc)

        action_name = gcal_event['summary']
        factory_name = '/' + action_name + "_create"
        try:
            factory = rospy.ServiceProxy(factory_name, CreateTask)
            # if 'description' in gcal_event:
            #     t = factory.call(gcal_event['description']).task
            # else:
            start_after = rospy.Time.from_sec(timegm(start_utc.timetuple())) \
                          - self.time_offset
            end_before = rospy.Time.from_sec(timegm(end_utc.timetuple())) \
                         - self.time_offset
            sa = "start_after: {secs: %d, nsecs: %d}" % \
                 (start_after.secs, start_after.nsecs)
            eb = "end_before: {secs: %d, nsecs: %d}" % \
                 (end_before.secs, end_before.nsecs)
            sn = "start_node_id: '%s'" % gcal_event['location']
            en = "end_node_id: '%s'" % gcal_event['location']
            if gcal_event.has_key('description'):
                ds = "description: '%s'" % gcal_event['description']
            else:
                ds = "description: "

            yaml = "{%s, %s, %s, %s, %s}" % (sa, eb, sn, en, ds)

            rospy.loginfo("calling with pre-populated yaml: %s" % yaml)
            t = factory.call(yaml).task
            rospy.loginfo("got the task back: %s" % str(t))
        except Exception as e:
            rospy.logwarn("Couldn't instantiate task from factory %s."
                          "error: %s."
                          "Using default constructor." %
                          (factory_name, str(e)))
            t = Task()
            t.action = gcal_event['summary']
            t.start_after = rospy.Time.from_sec(
                timegm(start_utc.timetuple())) \
                - self.time_offset
            t.end_before = rospy.Time.from_sec(timegm(end_utc.timetuple())) \
                - self.time_offset

        if 'location' in gcal_event:
            t.start_node_id = gcal_event['location']
            if len(t.end_node_id) == 0:
                t.end_node_id = gcal_event['location']
        if t.max_duration.secs == 0:
            t.max_duration = (t.end_before - t.start_after) / 2

        # if it's a time critical task, then the new
        # scheduler requires the task to have the same end
        # time as start time, to indicate time "criticalness".
        # Opportunistically, in this case we assume the
        # max duration to be the event length in calendar.
        if self.time_critical:
            t.max_duration = t.end_before - t.start_after
            t.max_duration.secs = t.max_duration.secs / 2
            t.end_before = t.start_after

        return t

    def _to_task_list(self):
        self.events = {}
        for gcal_event in self.gcal['items']:
            try:
                k = gcal_event['id'] + gcal_event['updated']
                self.events[k] = gcal_event
            except Exception as e:
                rospy.logerr('failed to convert event from iCal to task: %s',
                             str(e))

if __name__ == '__main__':
    t = Task()
    pprint(t)
