
import numpy as np

plus_scale = np.array([1,1,0])
mult_scale = np.array([10,10,1])
adjust_scale = np.array([2.5,0,0])

def up_scale( position ):
  new_pos = position.copy()

  new_pos += plus_scale
  new_pos *= mult_scale
  new_pos -= adjust_scale

  return new_pos

def down_scale( position ):
  new_pos = position.copy()

  new_pos += adjust_scale
  new_pos /= mult_scale
  new_pos -= plus_scale

  return new_pos

def bound_value(value, pattern):
  """
  Limit a value about a pattern of values

  Ex: Bound a value in 10 by 10 values

      bound_value(2, 10) -> 0
      bound_value(7, 10) -> 10
      bound_value(12, 10) -> 10
      bound_value(17, 10) -> 20

  Params:

    value   : Value to be bounded
    pattern : Pattern value that will be
              used as reference

  Return:

     The bounded value
  """

  value = round( value )
  rest = value % pattern

  if value < (pattern / 2):
    return 0
  elif rest <= ( pattern / 2 ):
    return value - rest
  elif value >= (pattern / 2) and value <= pattern:
    return pattern
  else:
    return value + ( pattern - rest )

def in_range(value, min_value, max_value, margin = 0.0):
  """
  Test if a value is in a range

  Params:

    value : Value to be tested
    min_value : Minimal value
    max_value : Maximal value
    margin : A margin in both side of the test

  Return:
    True  : if in range
    False : otherwise
  """

  return (min_value - margin) <= value <= (max_value + margin)

def rotation_matrix_z(deg):
  """
  Create a rotation matrix on Z axis

  Params:
    deg : Rotation in radians

  Return:
    The rotation matrix
  """

  return np.array([
    [ np.cos(deg), -np.sin(deg), 0 ],
    [ np.sin(deg),  np.cos(deg), 0 ],
    [ 0          ,  0          , 1 ]])

def rotation_matrix_x(deg):
  """
  Create a rotation matrix on X axis

  Params:
    deg : Rotation in radians

  Return:
    The rotation matrix
  """

  return np.array([
    [ 1,           0,            0 ],
    [ 0, np.cos(deg), -np.sin(deg) ],
    [ 0, np.sin(deg),  np.cos(deg) ]])

def rotation_matrix_y(deg):
  """
  Create a rotation matrix on Y axis

  Params:
    deg : Rotation in radians

  Return:
    The rotation matrix
  """

  return np.array([
    [  np.cos(deg), 0, np.sin(deg) ],
    [            0, 1,           0 ],
    [ -np.sin(deg), 0, np.cos(deg) ]])

def rotate_points(anchor_point, points, angle):
  """
  Rotate a set of point based on a anchor point
  along the Z axis by the passed angle in radians

  Params:
    anchor_point : Point used as anchor for the
                   rotation process
    points       : Set of point the be rotated
    angle        : Angle of the rotation

  Return:
    The new set of points rotated
  """

  # return np.array( points )

  points = np.array( points )
  anchor_point = np.array( anchor_point )
  mat = rotation_matrix_z( angle )

  new_points = []

  for point in points:
    temp_point = point - anchor_point
    temp_point = temp_point.dot( mat )
    temp_point += anchor_point

    new_points.append( temp_point )

  return np.array( new_points )
