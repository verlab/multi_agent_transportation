import math
import numpy as np
from tf.transformations import euler_from_quaternion

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def rad_to_deg(rad):
  return rad * 180 / np.pi

def deg_to_rad(deg):
  return deg * np.pi / 180

def angle_by_point(p1, p2):

  x = p2[0] - p1[0]
  y = p2[1] - p1[1]

  rad = math.atan( y / x )

  if x < 0: rad += np.pi

  rad = cut_angle( rad )

  deg = rad_to_deg(rad)

  return deg

def distance_by_point(p1, p2):
  source = np.array( p1 )
  target = np.array( p2 )

  return float( np.sqrt(((source - target) ** 2).sum()) )

def cut_angle(angle):
  while angle > np.pi:
    angle -= 2 * np.pi

  while angle < -np.pi:
    angle += 2 * np.pi

  return angle

def cut_angle_deg(angle):

  while angle > 180:
    angle = angle - 360

  while angle < -180:
    angle = angle + 360

  return angle

def constrain_value(value, constrain, limiar):

  if abs( value ) < limiar:
    return 0

  if value > -constrain and value < 0:
    return -constrain

  if value < constrain and value > 0:
    return constrain

  return value

def simplify_pose(pose):
  quat = [pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w ]

  euler = euler_from_quaternion(quat)

  simple_pose = {}
  simple_pose["theta"] = euler[2]
  simple_pose["x"] = pose.position.x
  simple_pose["y"] = pose.position.y
  simple_pose["z"] = pose.position.z

  return simple_pose
