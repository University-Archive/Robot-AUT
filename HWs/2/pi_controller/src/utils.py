import tf
import math


def quaternion_to_euler(msg):
  quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
  return (roll, pitch, yaw)


def distance(src, dest):
  return math.sqrt(((src[0] - dest[0]) ** 2) + ((src[1] - dest[1]) ** 2))


def gen_oval_path(w, h):
  path = []
  duration = 50
  for i in range(duration):
    t = 2 * math.pi * i / duration
    x = (w / 2) * math.cos(t)
    y = - (h / 2) * math.sin(t)
    path.append((x, y))
  return path

def gen_spiral_path(b):
  path = []
  duration = 70
  a = 1
  for i in range(duration):
    t = 4 * math.pi * i / duration
    r = a + b * t
    x = r * math.cos(t)
    y = - r * math.sin(t)
    path.append((x, y))
  return path