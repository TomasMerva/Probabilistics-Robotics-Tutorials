#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style


from gazebo_msgs.msg import ModelStates


class Visualizer:
  def __init__(self):
    self.fig, self.ax = plt.subplots()
    self.ln, = plt.plot([], [], 'r')
    self.x_data, self.y_data = [], []

  def plot_init(self):
    self.ax.set_xlim(-7, 7)
    self.ax.set_ylim(-7, 7)
    self.ln.set_linewidth(2)

  def OdometryCallback(self, msg):
    self.y_data.append(msg.pose.pose.position.x)
    self.x_data.append(msg.pose.pose.position.y)

  def update_plot(self, frame):
    self.ln.set_data(self.x_data, self.y_data)
    return self.ln

  def TrueOdometryCallback(self, msg):
    print( msg.pose[1].position.x )

vis = Visualizer()
rospy.init_node("visualize_trajectory")
sub = rospy.Subscriber("/odom", Odometry, vis.OdometryCallback)
sub_gazebo = rospy.Subscriber("/gazebo/model_states", ModelStates, vis.TrueOdometryCallback)

ani = animation.FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval=50)
plt.show(block=True)
