#!/usr/bin/env python
 
import socket
import os
import shutil
import subprocess
import signal

import time

import rospkg

from threading import Lock



class PrismClient(object):
    
    def __init__(self,port,dir_name):
        HOST = "localhost"
        PORT = port
        rospack = rospkg.RosPack()
        pack_dir=rospack.get_path('mdp_plan_exec')
        os.chdir(pack_dir + '/prism_robots/prism')
        os.environ['PRISM_MAINCLASS'] = 'prism.PrismManager'
        self.java_server=subprocess.Popen(["bin/prism",str(PORT),dir_name])
        time.sleep(5)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.command=''
        self.directory = dir_name 
        #self.directory=os.path.expanduser("~") + '/tmp/prism' + dir_name
        
        self.lock=Lock()
        
        
    def add_model(self,time_of_day, model_file):
        try:
            os.mkdir(self.directory + '/' + time_of_day)
        except OSError as ex:
            print 'error creating directory:',  ex
            
        command='add\n'
        command=command+time_of_day+'\n'
        command=command+model_file+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        print "added model:", data
        
        
        
    def check_model(self,time_of_day,specification):
        command='check\n'
        command=command+time_of_day+'\n'
        command=command+specification+'\n'
        print command
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        return data

        
    
    def get_policy(self,time_of_day,specification):
        command='plan\n'
        command=command+time_of_day+'\n'
        command=command+specification+'\n'
        print command
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        print "policy:", data
        return data
        
        
    def update_model(self,time_of_day,model_file):
        command='update\n'
        command=command+time_of_day+'\n'
        command=command+model_file+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        print "updated model:", data
       
       
    def shutdown(self,remove_dir):
        if remove_dir:
            shutil.rmtree(self.directory)        
            print 'temp dir removed'
        command='shutdown\n'
        self.lock.acquire()
        self.sock.sendall(command)
        self.sock.close()
        self.lock.release()
        print "Socket closed"
        os.kill(self.java_server.pid, signal.SIGHUP)
        #self.java_server.send_signal(signal.SIGHUP)

        
