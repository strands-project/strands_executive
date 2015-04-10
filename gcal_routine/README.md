# Google Calendar Routine

This package reads dates from a [Google Calendar](google.com/calendar/) and submits them to the scheduling service via the [`AddTasks`](https://github.com/strands-project/strands_executive/blob/hydro-devel/strands_executive_msgs/srv/AddTasks.srv) service. 

This requires the [`task_scheduler`](https://github.com/strands-project/strands_executive#running-scheduled-patrols) to be running, including its requirements. 

Start it with `rosrun gcal_routine gcal_routine_node.py`. You may have to also enable task execution via `rosservice call /task_executor/set_execution_status "status: true"`

The `gcal_routine` submits task to the scheduler queried from Google calendar using an API key via the official Google REST API. Goggle Calendar resolves all recurrences and this node gets a "linearised" view of events. By default, the event **title** of the event on GCal describes the *action* and the **Where** describes the start and end *place id*. Further arguments of the [`Task`](https://github.com/strands-project/strands_executive/blob/hydro-devel/strands_executive_msgs/msg/Task.msg) structure can be set in YAML in the **description** of the event. 
In addition, to make the user experience easier, tasks can be configured via a predefined template YAML file like this:

```
{
    'wait': {
        'action': 'wait_action',
        'max_duration': {secs: 300, nsecs: 0},
    },
    'patrol': {
        'action': 'wait_action',
        'max_duration': {secs: 30, nsecs: 0}
    }
}
```

This allows to define pre-defined values depending on the *action* entered in the **title** of the event. E.g. if `wait` is entered, then the attributes of the [`Task`](https://github.com/strands-project/strands_executive/blob/hydro-devel/strands_executive_msgs/msg/Task.msg) structure are being automatically populated from the template file.

## The following parameters are being recognised:
* `~calendar`: The ID of the google calendar, defaults to`henry.strands%40hanheide.net` (find the correct calendar ID in Calendar setting on Google Calendar)
* `~key`: The private access key to gain access to GCal, defaults to `AIzaSyC1rqV2yecWwV0eLgmoQH7m7PdLNX1p6a0` (This is the App key for the `henry.strands@hanheide.net` account. You may want to use other accounts.)
* `~available_tasks_conf_file`: The path to a yaml file providing the template as shown above
* `~pre_start_window_min`: Number of minutes that tasks are submitted to the scheduler *prior* to their `start_after` time. (default: 15 minutes)
* `routine_update_wait_sec`: seconds to sleep between checks for new tasks to be valid to be sent to the scheduler (default: 10s)
* `~gcal_poll_wait_sec`: seconds between polls to the Google calendar. Default is 60s (i.e. 1 poll per minute). You don't want this to be too small or Google will ban the account due to high bandwidth. Once per minute is a good start.
* `~priority`: Priority to assign to task. Default is 2.


All events in a Google calendar are translated into UTC time for the generation of tasks. So it should work in any time zone. 

Here is an example call:

```
rosrun  gcal_routine gcal_routine_node.py _stand_alone_test:=true _calendar:="hanheide.net_6hgulf44ij7ctjrf2iscj0m24o@group.calendar.google.com" _pre_start_window_min:=30 _gcal_poll_wait_sec:=10 _routine_update_wait_sec:=1
```