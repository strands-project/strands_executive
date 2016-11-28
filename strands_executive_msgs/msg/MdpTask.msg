strands_executive_msgs/MdpDomainSpec mdp_spec

# Priority of a task - higher number = higher priority
uint64 priority

# can the task be interrupted by demanded tasks
bool is_interruptible

# The task cannot start before this time
time start_after

# The task be completed after this time
time end_before
