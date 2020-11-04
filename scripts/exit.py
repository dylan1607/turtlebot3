#!/usr/bin/env python

import subprocess
import os

if __name__ == '__main__':
    
    subprocess.Popen('killall -9 rosmaster roscore python', shell=True)
    subprocess.Popen('pkill -9 python', shell=True)