#!/usr/bin/env python
 
import socket
import os
import shutil
import subprocess
import signal

import time

import rospkg



class PrismClient(object):
    
    def __init__(self):
        HOST = "localhost"
        PORT = 8085
        rospack = rospkg.RosPack()
        pack_dir=rospack.get_path('mdp_plan_exec')
        os.chdir(pack_dir + '/prism-robots/prism')
        os.environ['PRISM_MAINCLASS'] = 'prism.PrismManager'
        self.java_server=subprocess.Popen("bin/prism")
        time.sleep(5)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.command=''
        self.directory = os.path.expanduser("~") + '/tmp/prism'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        
        
    def add_model(self,time_of_day, model_file):
        try:
            os.mkdir(self.directory + '/' + time_of_day)
        except OSError as ex:
            print 'error creating directory:',  ex
            
        command='add\n'
        command=command+time_of_day+'\n'
        command=command+model_file+'\n'
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        print "added model:", data
        
        
        
    def check_model(self,time_of_day,specification):
        command='check\n'
        command=command+time_of_day+'\n'
        command=command+specification+'\n'
        print command
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        return data

        
    
    def get_policy(self,time_of_day,specification):
        command='plan\n'
        command=command+time_of_day+'\n'
        command=command+specification+'\n'
        print command
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        print "policy:", data
        
        
    def shutdown(self,remove_dir):
        if remove_dir:
            shutil.rmtree(self.directory)        
            print 'temp dir removed'
        command='shutdown\n'
        self.sock.sendall(command)
        self.sock.close()
        print "Socket closed"
        os.kill(self.java_server.pid, signal.SIGHUP)
        #self.java_server.send_signal(signal.SIGHUP)

        
