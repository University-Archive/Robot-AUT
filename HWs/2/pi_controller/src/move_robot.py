#! /usr/bin/env python

from xmlrpc.client import Error
import rospy
import tf 
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from utils import quaternion_to_euler, distance, gen_oval_path, gen_spiral_path
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class MoveRobot():
  def __init__(self):
    rospy.init_node("move_robot", anonymous=False)

    rospy.wait_for_service("/gazebo/set_model_state")
    state_msg = ModelState()
    state_msg.model_name = "turtlebot3_burger"
    state_msg.pose.position.x = 1
    state_msg.pose.position.y = 2
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    resp = set_state(state_msg)
    
    rospy.on_shutdown(self.shutdown)


    # hyper parameters
    self.k_theta = 0.45
    self.margin = 1
    self.window = 5
    self.k_p = 0.15
    self.k_i = 0.02

    # robot values
    self.x = 0
    self.y = 0
    self.theta = 0
    self.target_index = 0
    
    self.E = []

    self.is_oval = False
    if self.is_oval:
      self.init_oval()
    else:
      self.init_spiral()

    self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odometry)

    self.tick_pub = rospy.Publisher("/tick", Twist, queue_size=1)
    
    self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    self.vel = Twist()
    self.vel.linear.x = 0  # m/s
    self.vel.angular.z = 0  # rad/s

    self.finish_pub = rospy.Publisher("/finish", Twist, queue_size=5)

    self.rate = rospy.Rate(5)

    print("Target: ", self.target_index, self.points[self.target_index])

  def init_oval(self):
    self.d_star = 0.1
    self.w = 1
    self.h = 3
    self.points = gen_oval_path(self.w, self.h)

  def init_spiral(self):
    self.d_star = 0.25
    self.growth_factor = 0.1
    self.points = gen_spiral_path(self.growth_factor)

  def callback_odometry(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    (roll, pitch, yaw) = quaternion_to_euler(msg)
    self.theta = yaw

    self.check_target()

  def check_target(self):
    old_target = self.target_index
    target = self.points[self.target_index]

    if self.target_index == len(self.points) - 1:
      self.target_index = 0
      self.finish_pub.publish(self.vel)
      rospy.signal_shutdown("Finish")
    elif distance((self.x, self.y), target) < self.d_star:
      self.target_index += 1
    elif abs(self.x) > self.margin or abs(self.y) > self.margin:
      min_dist = distance((self.x, self.y), target)
      next_threshold = self.target_index + 3
      if next_threshold > len(self.points):
        next_threshold = len(self.points)
      for i in range(self.target_index + 1, next_threshold):
        dist = distance((self.x, self.y), self.points[i])
        if dist < min_dist:
          min_dist = dist
          self.target_index = i
          break

    if self.target_index != old_target:
      print("Target: ", self.target_index, self.points[self.target_index])

  def send_velocity_cmd(self):
    target = self.points[self.target_index]
    delta_x = target[0] - self.x
    delta_y = target[1] - self.y

    theta_star = math.atan2(delta_y, delta_x)
    alpha = theta_star - self.theta
    if alpha > math.pi:
      alpha -= 2 * math.pi
    if alpha < -math.pi:
      alpha += 2 * math.pi
    self.vel.angular.z = self.k_theta * alpha

    e = math.sqrt(delta_x ** 2 + delta_y ** 2)
    if e < 0.5 and self.target_index == 0:
      self.tick_pub.publish(self.vel)
      self.k_i = 0.17
      self.k_p = 0.43
    e_p = e - self.d_star
    if len(self.E) < self.window:
      self.E.append(e_p)
    else:
      self.E.pop(0)
      self.E.append(e_p)
    v = (self.k_p * e_p) + (self.k_i * np.array(self.E).sum())
    self.vel.linear.x = v

    self.vel_pub.publish(self.vel)
    self.tick_pub.publish(self.vel)

  def shutdown(self):
    rospy.loginfo("Stop TurtleBot")
    self.vel.linear.x = 0
    self.vel.angular.z = 0
    self.vel_pub.publish(self.vel)
    rospy.sleep(1)


if __name__ == "__main__":
  try:
    controller = MoveRobot()
    while not rospy.is_shutdown():
      controller.send_velocity_cmd()
      controller.rate.sleep()
  except:
    rospy.loginfo("move_robot node terminated")
