import tf
import math


def quaternion_to_euler(msg):
  quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
  return (roll, pitch, yaw)


def distance(src, dest):
  return math.sqrt(((src[0] - dest[0]) ** 2) + ((src[1] - dest[1]) ** 2))


points = [(1.5, 0), (1.5, -0.5), (1.5, -1), (1.5, -1.5),
          (1, -1.5), (0.5, -1.5), (0, -1.5), (-0.5, -1.5),
          (-1, -1.5), (-1.5, -1.5), (-1.5, -1), (-1.5, -0.5),
          (-1.5, 0), (-1.5, 0.5), (-1.5, 1), (-1.5, 1.5),
          (-1, 1.5), (-0.5, 1.5), (0, 1.5), (0.5, 1.5),
          (1, 1.5), (1.5, 1.5), (1.5, 1), (1.5, 0.5)]
