#!/usr/bin/env python
# coding:utf-8
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix #导入消息
import time

x = 0
y = 0

def callback(data):
    global x
    global y
    x = (data.latitude - 49.9) * 111000
    y = -(data.longitude - 8.9) * 71950
    #rospy.loginfo(str(x) + "   " + str(y))

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
    rospy.Subscriber("/sensor_msgs/NavSatFix", NavSatFix,callback)
    fig = plt.figure()
    while(1):
        add_queue(x, y)
        plt.scatter(x_list, y_list,marker = 'o', color = 'g', label='position', s = 10)
        plt.xlim(0,5)
        plt.ylim(-2.5,2.5)
        plt.pause(0.001)
        plt.clf()
        rospy.loginfo(str(x) + " " + str(y))
    rospy.spin()

if __name__ == '__main__':
    show_traject()
