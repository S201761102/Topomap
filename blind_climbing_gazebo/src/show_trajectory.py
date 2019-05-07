#!/usr/bin/env python
# coding:utf-8
import sys
import signal
import time
import rospy
import matplotlib.pyplot as plt
import numpy as np
from multisensor_information_fusion.msg import pose #导入消息

x = 0
y = 0
psi = 0

def callback(data):
    global x
    global y
    global psi
    x = data.x
    y = data.y
    psi = data.psi
    rospy.loginfo(str(x) + "   " + str(y) + "   "+str(psi*180.0/3.1415926))

def quit(signum, frame):
    print ''
    print 'stop showing'
    sys.exit()

x_list = []
y_list = []
queue_max_len = 100
def add_queue(x, y):
    if len(x_list) < queue_max_len:
        x_list.append(x)
        y_list.append(y)
    else:
        x_list.pop(0)
        y_list.pop(0)
        x_list.append(x)
        y_list.append(y)
        print len(x_list)

def show_traject():
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pose", pose,callback)
    fig = plt.figure()
    plt.grid(True)
    while 1:
        U = float(np.cos(psi)*0.5)
        V= float(np.sin(psi)*0.5)
        plt.quiver(x,y,U,V)
        plt.scatter(x_list, y_list,marker = 'o', color = 'g', label='position', s = 10)
        plt.xlim(-1,8)
        plt.ylim(-1,8)
        plt.pause(0.01)
        plt.clf()
    #rospy.spin()

if __name__ == '__main__':
    show_traject()
