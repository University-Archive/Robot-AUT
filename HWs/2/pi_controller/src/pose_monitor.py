#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from utils import distance, gen_oval_path, gen_spiral_path
import math

class PoseMonitor():
  def __init__(self):
    rospy.init_node("pose_monitor", anonymous=True)
    rospy.on_shutdown(self.shutdown)

    self.report_pose = False

    self.x = []
    self.y = []

    self.last_round_index = 0

    self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odometry)
    self.tick_sub = rospy.Subscriber(
        "/tick", Twist, self.callback_velocity_change)
    self.finish_sub = rospy.Subscriber("/finish", Twist, self.report)

    print("Wait for service ....")
    rospy.wait_for_service("gazebo/get_model_state")
    print("Got it!")

    self.get_ground_truth = rospy.ServiceProxy(
        "gazebo/get_model_state", GetModelState)

  def callback_velocity_change(self, msg):
    self.report_pose = True

  def callback_odometry(self, msg):
    if self.report_pose:
      x = round(msg.pose.pose.position.x, 2)
      y = round(msg.pose.pose.position.y, 2)

      if len(self.y) > 0  and y * self.y[-1] < 0 and self.x[-1] > 0:
        self.last_round_index = len(self.y)

      self.x.append(x)
      self.y.append(y)

      print("Position: (%5.2f, %5.2f)" %
            (msg.pose.pose.position.x, msg.pose.pose.position.y))
      print("Ground Truth: ", self.get_ground_truth(
          "turtlebot3_burger", "world").pose)

      self.report_pose = False

  def calc_error(self):
    is_oval = False
    if is_oval:
      path = gen_oval_path(1, 3)
    else:
      path = gen_spiral_path(0.1)
    error = 0
    for i in range(len(self.x)):
      min_dist = math.inf
      for j in path:
        dist = distance((self.x[i], self.y[i]), j)
        if dist < min_dist:
          min_dist = dist
      error += min_dist
    print(f"Error: {error / len(self.x)}")
    return path, (error / len(self.x))


  def report(self, msg):
    self.report_pose = False
    path, error = self.calc_error()
    path = np.array(path)
    fig, ax = plt.subplots()
    plt.ylabel("Y")
    plt.xlabel("X")
    plt.title("TurtleBot")
    plt.scatter(path[:, 0], path[:, 1], c="red", s=5)
    ax.plot(self.x, self.y)
    plt.xlim([-3, 3])
    plt.ylim([-3, 3])
    plt.savefig('foo.png')
    rospy.signal_shutdown("Finish")

  def shutdown(self):
    rospy.loginfo("Stop Monitor")
    rospy.sleep(1)


if __name__ == "__main__":
  try:
    while not rospy.is_shutdown():
      PoseMonitor()
      rospy.spin()
  except:
    rospy.loginfo("move_robot node terminated")
