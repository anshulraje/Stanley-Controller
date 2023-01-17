#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

# def animate(i):
#     ax1.clear()
#     ax1.plot(x_arr, y_arr)

# plt.show()

x_arr = []
y_arr = []

path_x = []
path_y = []

path_x_new = []
path_y_new = []

def odom_callback(odometry: Odometry):
    global x_arr
    global y_arr
    x_arr.append(odometry.pose.pose.position.x)
    y_arr.append(odometry.pose.pose.position.y)

def path_callback(path: Path):
    global path_x, path_y
    poses = path.poses
    for index, poseStamped in enumerate(poses):
        path_x.append(poseStamped.pose.position.x) 
        path_y.append(poseStamped.pose.position.y)
        # if index % 6 == 0:
        #     path_x_new.append(poseStamped.pose.position.x) 
        #     path_y_new.append(poseStamped.pose.position.y)

if __name__ == '__main__':
    
    rospy.init_node('live_plotter')

    odom_subscriber = rospy.Subscriber('/odom', Odometry, callback=odom_callback)
    path_subscriber = rospy.Subscriber('/path', Path, callback=path_callback)

    def animate(i):
        ax1.clear()
        plt.xlim([-1, 7])
        plt.ylim([-2, 2])
        for theta in range(0, 628, 50):
            x = theta/100
            y = math.sin(x)
            ax1.plot(x, y, marker='.', markersize=5, c='y')
        # ax1.plot(1.57, 1, 'yo')
        ax1.plot(x_arr, y_arr, c='black')
        # ax1.plot()
        

    ani = animation.FuncAnimation(fig, animate, interval=500)
    plt.show()

