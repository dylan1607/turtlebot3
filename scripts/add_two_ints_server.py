#!/usr/bin/env python

from __future__ import print_function
from spc.srv import AddTwoInts
import rospy

def handle_add_two_ints(req):
    return (req.a + req.b)

def add_two_ints_server():
    rospy.init_node('Add_Intsance')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print('Server Run OK')
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()