# logging object for task events

uint8 ADDED=1
uint8 DEMANDED=2
uint8 TASK_STARTED=3

uint8 NAVIGATION_STARTED=4
uint8 NAVIGATION_SUCCEEDED=5
uint8 NAVIGATION_FAILED=6
uint8 NAVIGATION_PREEMPTED=7

uint8 EXECUTION_STARTED=8
uint8 EXECUTION_SUCCEEDED=9
uint8 EXECUTION_FAILED=10
uint8 EXECUTION_PREEMPTED=11

uint8 CANCELLED_MANUALLY=12
uint8 DROPPED=13

# Finished was used by the early versions of executor
uint8 TASK_FINISHED=14

# These three are more modern
uint8 TASK_FAILED=15
uint8 TASK_SUCCEEDED=16
# TASK_PREEMPTED is what happens when the task action server is pre-empted, i.e. the execution of the task server must have started 
uint8 TASK_PREEMPTED=17
# TASK_STOPPED is what happens when the task action server is never started, but navigation has started for the task. This is only used on the mdp executor where a task with this status was included in the execution batch but never actually run. This means the task was retained for later execution
uint8 TASK_STOPPED=18


# Used for the routine
uint8 ROUTINE_STARTED=30
uint8 ROUTINE_STOPPED=31
uint8 ROUTINE_ADDED=32

Task task
uint8 event
time time
string description
