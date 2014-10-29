#!/bin/bash

N=$1
VERSION=$2

echo rosrun scheduler scheduler_node _save_problems:=false _scheduler_version:=$VERSION _output_file:=$HOME/$VERSION-$N.txt _timeout:=180  get_schedule:=get_schedule_$N __name:=scheduler_node_$N
rosrun scheduler scheduler_node _save_problems:=false _scheduler_version:=$VERSION _output_file:=$HOME/$VERSION-$N.txt _timeout:=180  get_schedule:=get_schedule_$N __name:=scheduler_node_$N
