#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from utils import distance


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
    error = 0
    for i in range(self.last_round_index, len(self.x)):
      if abs(self.x[i]) > 1.5 and abs(self.y[i]) > 1.5:
        error += distance((1.5, 1.5), (self.x[i], self.y[i]))
      elif abs(self.x[i]) > abs(self.y[i]):
        error += abs(abs(self.x[i]) - 1.5)
      else:
        error += abs(abs(self.y[i]) - 1.5)
    print(f"Error: {error / (len(self.x) - self.last_round_index - 1)}")


  def report(self, msg):
    self.report_pose = False
    fig, ax = plt.subplots()
    plt.ylabel("Y")
    plt.xlabel("X")
    plt.title("TurtleBot")
    ax.add_patch(Rectangle((-1.5, -1.5), 3, 3, edgecolor="red", fill=False))
    ax.plot(self.x, self.y)
    plt.xlim([-3, 3])
    plt.ylim([-3, 3])
    print(len(self.x))
    plt.savefig('foo.png')
    self.calc_error()
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
