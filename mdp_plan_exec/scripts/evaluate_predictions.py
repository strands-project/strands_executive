#!/usr/bin/env python
# -*- coding: utf-8 -*-

from strands_navigation_msgs.msg import *
from strands_navigation_msgs.srv import *

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import actionlib
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from strands_executive_msgs.srv import *
from strands_navigation_msgs.msg import NavStatistics
from datetime import datetime, timedelta
from task_executor import task_routine, task_query
import argparse
import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta, time, date
from task_executor import task_routine, task_query
from task_executor.utils import rostime_to_python, python_to_rostime
from task_executor.task_query import task_groups_in_window, daily_windows_in_range, print_group
import pytz
from dateutil.relativedelta import *
import matplotlib.patches as mpatches
import numpy as np
import matplotlib.pyplot as plt

import argparse
import cmd

import pymongo
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

edge_id_to_x = {}

def visualise_window(result_time, predictor, results_collection, window_start, window_end, colour = 'b', label = None):
    # get all results for this set
    # and build up answers by edge
    edge_results = {}
    query =  {'result_time': result_time, 'predictor': predictor, 'training_window_start': window_start, 'training_window_end': window_end}

    print window_start

    for result in results_collection.find(query):
        edge_id = result['start'] + '_' + result['end']
        for error in result['results']:
            if edge_id not in edge_results:
                edge_results[edge_id] = []    

            if edge_id not in edge_id_to_x:
                edge_id_to_x[edge_id] = (len(edge_id_to_x) + 1) * 10

            edge_results[edge_id].append(error)


    # now we have a list of errors per edge
    edge_ids = []
    edge_means = []
    edge_stddevs = []
    for edge_id, edge_errors in edge_results.iteritems():
        edge_errors = np.absolute(np.array(edge_errors))
        edge_ids.append(edge_id_to_x[edge_id])
        edge_means.append(edge_errors.mean())
        edge_stddevs.append(edge_errors.std())


    # print edge_ids, edge_means, edge_stddevs        

    plt.errorbar(edge_ids, edge_means, edge_stddevs, linestyle='None', marker='^', color = colour, label = label)
   

def visualise_results(tmap, result_time):
    # first reconstruct windows from the data

    host = rospy.get_param('mongodb_host', 'localhost')
    port = rospy.get_param('mongodb_port', '62345')
    client = pymongo.MongoClient(host,port)    
    
    results_collection = client['mdp_results'][tmap]

    predictors = results_collection.find({'result_time': result_time}, {'predictor': 1})
    predictors = set([predictor['predictor'] for predictor in  predictors])


    # gather in the unique windows that defined the training regime

    predictor_windows = []
    for predictor in predictors:        
        print predictor
        window_dates = {}
        for window in results_collection.find({'result_time': result_time, 'predictor': predictor}, {'training_window_start': 1, 'training_window_end':1}): 
            if window['training_window_start'] not in window_dates:
                window_dates[window['training_window_start']] = []
            window_dates[window['training_window_start']].append(window['training_window_end'])

        windows = []
        for start, ends in window_dates.iteritems():
            ends = set(ends)        
            windows += [[predictor, start, end] for end in ends] 


        predictor_windows += windows


    predictor_windows.sort(key=lambda w: w[2])
    predictor_windows.sort(key=lambda w: w[0])
    print predictor_windows


    colour_map = plt.get_cmap('Paired')
    colours = [colour_map(i * 30) for i in range(len(predictor_windows))]

    count = 0

    with PdfPages('{0}_mdp_stats.pdf'.format(tmap)) as pdf:
    
        fig = plt.figure(figsize=(40,5))
        ax = fig.add_subplot(1,1,1)
        ax.set_yscale('log')

        for window in predictor_windows:
            print window
            visualise_window(result_time, window[0], results_collection, window[1], window[2], colours[count], window[0] + str(count))
            count += 1

            for edge, x in edge_id_to_x.iteritems():
                edge_id_to_x[edge] = x + 1

        plt.legend()
        # plt.show()
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()

def model_client():
    #Creating monitored navigation client    
    rospy.loginfo("Getting model build server")
    client = actionlib.SimpleActionClient('/topological_prediction/build_temporal_model', BuildTopPredictionAction)
    client.wait_for_server()
    rospy.loginfo(" ...done")
    return client

def prediction_client():
    rospy.wait_for_service('/mdp_plan_exec/get_expected_travel_times_to_waypoint')
    return rospy.ServiceProxy('/mdp_plan_exec/get_expected_travel_times_to_waypoint', GetExpectedTravelTimesToWaypoint)


def close_in_time(prev, next, close=10):
    prev_end = prev[1]['epoch'] + prev[0].operation_time
    next_start = next[1]['epoch']
    # print abs(prev_end - next_start)
    return abs(prev_end - next_start) < close

def to_epoch(ptime):
    return int((ptime - datetime(1970,1,1, tzinfo=ptime.tzinfo)).total_seconds())

def python_to_rostime(ptime):
    return rospy.Time((ptime - datetime(1970,1,1, tzinfo=ptime.tzinfo)).total_seconds())

def evaluate_prediction(predictor, start, end, stats):
    try:

        # string target_waypoint     
        # time epoch                 
        # ---                        
        # string[] source_waypoints  
        # duration[] travel_times 

        # path_stats.append((path[0][0].origin,path[-1][0].final_node, path[0][1]['epoch'], sum([stat.operation_time for (stat, meta) in path])))

        print stats
        errors = []        
        for stat in stats:

            response = predictor(end, rospy.Time(stat[0]))            
            predicted_duration = response.travel_times[response.source_waypoints.index(start)]
            actual_duration = stat[1]
            errors.append(actual_duration - predicted_duration.to_sec())

        return errors

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def collect_predictions(predictor_name, now, tmap, training_window_start, training_window_end, training_window_size, training_windows, testing_window_start, testing_window_end, path_stats):

    builder = model_client()
    predictor = prediction_client()

    host = rospy.get_param('mongodb_host', 'localhost')
    port = rospy.get_param('mongodb_port', '62345')
    client = pymongo.MongoClient(host,port)    

    results_collection = client['mdp_results'][tmap]

    

    processed = 0
    increment = 100
    increment_count = 100

    for window in range(training_windows):

        window_start = training_window_start
        window_end = training_window_end + training_window_size * window

        build_goal = BuildTopPredictionGoal(python_to_rostime(window_start), python_to_rostime(window_end))
        builder.send_goal(build_goal)
        builder.wait_for_result()

        # now test each nav stat from the windows
        result_set = '%s___%s' % (window_start, window_end)
        for start, end_dict in path_stats.iteritems():

            for end, stats in end_dict.iteritems():
                results = evaluate_prediction(predictor, start, end, stats)
                result_doc = {'result_time': now, 'training_window_start': window_start, 'training_window_end': window_end, 'predictor' : predictor_name, 'results': results, 'start': start, 'end': end} 
                results_collection.insert(result_doc)
                processed += len(results)

                if processed > increment_count:
                    print 'Processed %s/%s' % (processed, len(path_stats) * training_windows)
                    increment_count += increment

    print 'Processed %s/%s' % (processed, len(path_stats) * training_windows)

    return now


def do_g4s_predictions(path_stats):
    # get all nav stats
    tz = pytz.timezone(pytz.country_timezones['gb'][0])

    training_window_start = datetime(2015,5,6,8,0,tzinfo=tz)
    training_window_end = datetime(2015,5,12,18,0,tzinfo=tz)
    training_window_size = timedelta(days=7)
    training_windows = 1

    # manually move on, add two extra weeks
    training_window_end += training_window_size * 3

    testing_window_start = datetime(2015,6,4,8,0,tzinfo=tz)
    testing_window_end = datetime(2015,6,10,18,0,tzinfo=tz)

    tmap = 'g4s_tmap'

    collect_predictions('fremen', 1, tmap, training_window_start, training_window_end, training_window_size, training_windows, testing_window_start, testing_window_end, path_stats)

def group_paths(path_stats):
    grouped_paths = {}
    for path_stat in path_stats:
        # path_stats.append((path[0][0].origin,path[-1][0].final_node, path[0][1]['epoch'], sum([stat.operation_time for (stat, meta) in path])))
        if path_stat[0] not in grouped_paths:
            grouped_paths[path_stat[0]] = {path_stat[1]: [[path_stat[2],path_stat[3]]]}
        elif path_stat[1] not in grouped_paths[path_stat[0]]:
            grouped_paths[path_stat[0]][path_stat[1]] = [[path_stat[2],path_stat[3]]]
        else:
            grouped_paths[path_stat[0]][path_stat[1]].append([path_stat[2],path_stat[3]])

    return grouped_paths

def gather_paths(start, end):
    nav_store = MessageStoreProxy(collection="nav_stats")
    task_store = MessageStoreProxy(collection="task_events")

    paths = [[]]
    for task_group in task_groups_in_window(start, end, task_store, event=[TaskEvent.NAVIGATION_STARTED, TaskEvent.NAVIGATION_SUCCEEDED]):
        
        meta_query = {"inserted_at": {"$gte": rostime_to_python(task_group[0].time), "$lte" : rostime_to_python(task_group[-1].time)}}
        nav_stats = nav_store.query(NavStatistics._type, meta_query = meta_query)
        nav_stats.sort(key=lambda x: x[1]["epoch"])

        for (stat, meta) in nav_stats:        
            # if stat.status == 'success':
                # check they're joined up in space and time
            if stat.origin != 'none' and stat.final_node != 'none':
                if len(paths[-1]) > 0 and paths[-1][-1][0].final_node == stat.origin and close_in_time(paths[-1][-1], (stat, meta)):
                    paths[-1].append((stat, meta))
                # if not create a new path
                else:
                    paths.append([(stat, meta)])
            else:
                paths.append([])

        paths.append([])       


    min_path_length = 2

    paths = [path for path in paths if len(path) >= min_path_length]
    print len(paths)
    example_path = paths[-1]
    path_stats = []
    for path in paths:
        path_stats.append((path[0][0].origin,path[-1][0].final_node, path[0][1]['epoch'], sum([stat.operation_time for (stat, meta) in path])))

    path_lengths = [len(path) for path in paths]
    print '{0} paths, max len {1}'.format(len(path_stats), max(path_lengths))
    # n, bins, patches = plt.hist(path_lengths, bins = max(path_lengths))
    # plt.show()


    return group_paths(path_stats)


if __name__ == '__main__':

    # to get something that looks roughly like the last week to me
    # ./nav_paths.py "09/06/14 08:00"

    rospy.init_node("nav_path_prediction_evaluator")

    parser = argparse.ArgumentParser(description='Prints a summary of tasks executed within the queried time window.')
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', default=datetime.fromtimestamp(0),
                   help='start datetime of query, defaults to no start. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=task_query.mkdatetime, nargs='?', default=datetime.utcnow(),
                   help='end datetime of query, defaults to now. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    args = parser.parse_args()

    
    do_g4s_predictions(gather_paths(args.start, args.end))
    visualise_results('g4s_tmap', 1)


    

    #for path_stat in path_stats:
    #    print '%s to %s at %s took %s' % path_stat




