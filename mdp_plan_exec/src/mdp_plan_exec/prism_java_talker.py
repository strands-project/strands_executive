import rospy
import socket
import os
import shutil
import subprocess
import signal
from threading import Lock

class PrismJavaTalker(object):
    
    def __init__(self,port,dir_name,file_name):
        HOST = "localhost"
        PORT = port
        #prism_dir='/opt/prism-robots/prism'
        #prism_dir='/home/strands/bruno_ws/prism-robots/prism'
        prism_dir='/home/bruno/devel_ws/prism-robots/prism'
        os.chdir(prism_dir)
        os.environ['PRISM_MAINCLASS'] = 'prism.PrismPythonTalker'
        self.java_server=subprocess.Popen(["bin/prism",str(PORT),dir_name, file_name])
        rospy.sleep(1)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.directory = dir_name         
        self.lock=Lock()
        
    def check_model(self,specification):
        command='check\n'
        command=command+specification+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        return data

    def get_policy(self,specification, partial_sat=False):
        if partial_sat:
            command='partial_sat_plan\n'
        else:
            command='plan\n'
        command=command+specification+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        if partial_sat:
            rospy.loginfo("PAXPROB; EXPETECT TIME: " + data)
        else:    
            rospy.loginfo("Expected time from current node: " +  data)
        return data
    
    
    
    def get_state_vector(self,specification, is_partial=False):
        state_vector=[]
        if is_partial:
            command='partial_sat_get_vector\n'
        else:
            command='get_vector\n'
        command=command+specification+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        if data=="start\n":
            self.sock.sendall("ack\n")
        else:
            rospy.logwarn("socket error while getting state vector")
            self.sock.sendall("error")
        while True:
            data=self.sock.recv(1024)
            if data=="end\n":
                break
            try:
                state_vector.append(float(data))
                self.sock.sendall("ack\n")
            except ValueError, e:
                rospy.logwarn("socket error while getting state vector")
                self.sock.sendall("error\n")
        self.lock.release()
        return state_vector       
        
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
