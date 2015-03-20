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
        os.chdir('/opt/prism-robots/prism')
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

    def get_policy(self,specification):
        command='plan\n'
        command=command+specification+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        rospy.loginfo("Expected time from current node: " +  data)
        return data
    
    def get_state_vector(self,specification):
        state_vector=[]
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
        rospy.loginfo("Expected times to target state: " + str(state_vector))
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
