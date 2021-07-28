#! /usr/bin/env python

from xmlrpc.client import Error
import rospy
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from utils import quaternion_to_euler, distance, points


class MoveRobot():
  def __init__(self):
    rospy.init_node("move_robot", anonymous=False)
    rospy.on_shutdown(self.shutdown)

    # hyper parameters
    self.k_theta = 0.45
    self.d = 1
    self.margin = 2

    # robot values
    self.x = 0
    self.y = 0
    self.theta = 0
    self.target_index = 0

    self.round = 0
    print(f"Round {self.round + 1}")

    self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odometry)

    self.tick_pub = rospy.Publisher("/tick", Twist, queue_size=1)
    
    self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    self.vel = Twist()
    self.vel.linear.x = 0.9  # m/s
    self.vel.angular.z = 0  # rad/s

    self.finish_pub = rospy.Publisher("/finish", Twist, queue_size=5)

    self.rate = rospy.Rate(5)

    print("Target: ", self.target_index, points[self.target_index])
    self.tick_pub.publish(self.vel)

  def callback_odometry(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    (roll, pitch, yaw) = quaternion_to_euler(msg)
    self.theta = yaw

    self.check_target()

  def check_target(self):
    old_target = self.target_index
    target = points[self.target_index]

    if self.target_index == len(points) - 1:
      self.target_index = 0
      self.round += 1
      self.k_theta = 0.68
      self.margin = 1.8
      if self.round == 10:
        rospy.sleep(1.5)
        self.finish_pub.publish(self.vel)
        rospy.signal_shutdown("Finish")
        # pass
      else:
        print(f"Round {self.round + 1}")
    elif distance((self.x, self.y), target) < self.d:
      # self.d = 0.6
      self.target_index = (self.target_index + 1) % len(points)
    elif abs(self.x) > self.margin or abs(self.y) > self.margin:
      min_dist = distance((self.x, self.y), target)
      for i in range(self.target_index + 1, self.target_index + 4):
        i = i % len(points)
        dist = distance((self.x, self.y), points[i])
        if dist < min_dist:
          min_dist = dist
          self.target_index = i

    if self.target_index != old_target:
      print("Target: ", self.target_index, points[self.target_index])

  def send_velocity_cmd(self):
    target = points[self.target_index]

    theta_star = math.atan2(
        target[1] - self.y, target[0] - self.x)
    alpha = theta_star - self.theta
    if alpha > math.pi:
      alpha -= 2 * math.pi
    if alpha < -math.pi:
      alpha += 2 * math.pi

    self.vel.angular.z = self.k_theta * alpha

    self.vel_pub.publish(self.vel)
    self.tick_pub.publish(self.vel)

  def shutdown(self):
    rospy.loginfo("Stop TurtleBot")
    self.vel.linear.x = 0.0
    self.vel.angular.z = 0.0
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
