import rospy
import socket
import os
import shutil
import subprocess
import signal
from threading import Lock

class PartialSatPrismJavaTalker(object):
    
    def __init__(self,port,directory,file_name):
        self.HOST = "localhost"
        self.PORT = port
        self.prism_dir = '/opt/prism-robots/prism'
        self.directory = directory
        self.file_name = file_name
        self.lock=Lock()
        
        self.java_server = None
        self.sock = None
        self.timeout=60*8
        self.start()

    
    
    def start(self):
        #os.chdir(self.prism_dir)
        #os.environ['PRISM_MAINCLASS'] = 'prism.PartialSatPrismPythonTalker'
        #self.java_server=subprocess.Popen(["bin/prism", str(self.PORT), self.directory, self.file_name,  '-javamaxmem', '4g'])
        rospy.sleep(1)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.HOST, self.PORT))
        self.sock.settimeout(self.timeout)

    
    def call_prism(self,specification):
        command='partial_sat_guarantees\n'
        command=command+specification+'\n'
        self.lock.acquire()
        data = None
        try:
            self.sock.sendall(command)
            data = self.sock.recv(1024)
            self.lock.release()
        except Exception, e:
            rospy.logwarn("Error in socket communication: " + str(e))
            rospy.loginfo("Restarting PRISM")
            self.shutdown(False)
            rospy.sleep(5)
            self.start()
            self.lock.release()
         
        return data=="success\n"

        
    def shutdown(self,remove_dir=True):
        if remove_dir:
            shutil.rmtree(self.directory)        
            rospy.loginfo('prism temp dir removed')
        #command='shutdown\n'
        #
        #self.sock.sendall(command)
        self.sock.close()
        #
        rospy.loginfo("Socket closed")
        os.system("fuser -k  " + str(self.PORT) + "/tcp")
