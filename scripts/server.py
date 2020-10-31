#!/usr/bin/env python

import subprocess
import sys
import psutil
import signal
import rospy

class Start(object):
    __init = False
    def __init__(self):
        if Start.__init:
            raise Exception('Another Instance RosCore running !!')
        else:
            Start.__init =  True
            self.run()
    def run(self):
        try:
            self.roscore_process = subprocess.Popen('roscore')
            self.roscore_pid = self.roscore_process.pid
            rospy.sleep(1)
            self.rosweb = subprocess.Popen('roslaunch rosbridge_server rosbridge_websocket.launch', shell=True)
            self.rosweb_pid = self.rosweb.pid
            rospy.sleep(2)
            print('----------------Server Ready-----------------')
        except OSError as e:
            sys.stderr.write('Roscore could not run')
            self.terminal()
            raise e

    def terminal(self):
        self.kill_process(self.rosweb_pid)
        self.rosweb.terminate()
        self.rosweb.wait()
        self.kill_process(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()
        Start.__init = False

    def kill_process(self,pid,sig=signal.SIGTERM):
        try:
            parent = psutil.Process(pid)
            print('Process PID: ',pid)
            print(parent)
        except psutil.NoSuchProcess:
            print("parent process not existing")
            return
        children = parent.children(recursive=True)
        print(children)
        try:
            for process in children:
                process.send_signal(sig)
        except:
            pass

if __name__ == '__main__':
    try:
        Start()
    except:
        print('----------Server Not Running----------')