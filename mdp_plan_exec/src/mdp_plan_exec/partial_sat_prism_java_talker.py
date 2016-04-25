import rospy
import socket
import os
import shutil
import subprocess
import signal
from threading import Lock

class PartialSatPrismJavaTalker(object):
    
    def __init__(self,port,dir_name,file_name):
        HOST = "localhost"
        PORT = port
        prism_dir='/opt/prism-robots/prism'
        #prism_dir='/home/strands/bruno_ws/prism-robots/prism'
        #prism_dir='/home/bruno/devel_ws/prism-robots/prism'
        #prism_dir='/Users/nah/code/prism-robots/prism'
        os.chdir(prism_dir)
        os.environ['PRISM_MAINCLASS'] = 'prism.PartialSatPrismPythonTalker'
        self.java_server=subprocess.Popen(["bin/prism", str(PORT),dir_name, file_name,  '-javamaxmem', '4g'])
        rospy.sleep(1)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.directory = dir_name         
        self.lock=Lock()
        self.sock.settimeout(60*5)
        
    def call_prism(self,specification):
        command='partial_sat_guarantees\n'
        command=command+specification+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        return data=="success\n"

        
    def shutdown(self,remove_dir=True):
        if remove_dir:
            shutil.rmtree(self.directory)        
            rospy.loginfo('prism temp dir removed')
        command='shutdown\n'
        self.lock.acquire()
        self.sock.sendall(command)
        self.sock.close()
        self.lock.release()
        rospy.loginfo("Socket closed")
        os.kill(self.java_server.pid, signal.SIGHUP)
