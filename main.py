import time
from math import cos, sin, ceil
import copy



# odom to track current location
def math():
  deltaLE = 0 # change in left encoder
  deltaRE = 0 # change in right encoder value
  x_location = 0
  y_location = 0
  robo_angle = 0 # from gyro sensor

  distance = (deltaLE + deltaRE) / 2
  x_location += distance*cos(robo_angle)
  y_location += distance*sin(robo_angle)

  
def pathgen(segments: list):
  new_points = []
  spacing = 6
  for line_segment in segments:
    vector = line_segment[1] - line_segment[0]
    num_points_that_fit = ceil(len(vector) / spacing)
    vector = vector.normalize() * spacing
    for i in range(len(num_points_that_fit)):
      sol = line_segment + vector * i
      new_points.append(sol)
  new_points.append(segments[len(segments) - 1])

def smoothing(path: list, a, b, tolerance = 0.001):
  
  # larger b = smoother path. b should be between 0.75 and 0.78
  # a = 1 - b
  # tolerance defaults to 0.001
  new_path = copy.deepcopy(path)
  change = tolerance
  while change >= tolerance:
    change = 0
    for i in range(len(path) - 1):
      for j in range(path[i]):
        aux = new_path[i][j]
        new_path[i][j] += a * (path[i][j] - new_path[i][j]) + b * (new_path[i-1][j] + new_path[i+1][j] - (2.0 * new_path[i][j]))
        change += abs(aux - new_path[i][j])
  return new_path




# helper/calculation functions

def calc_distance(prev_point, cur_point):
  prev_distance = prev_point[1] - prev_point[0]
  x1 = cur_point[0]
  y1 = cur_point[1]
  x2 = prev_point[0]
  y2 = prev_point[1]
  return prev_distance + ((((x2 - x1 )**2) + ((y2-y1)**2) )**0.5)

def calc_curve(P, Q, R): # Q is our current point. P and R are the points Q is between. Hence, this algorithm is not applicable for the first and last points
  P[1] += 0.001
  k1 = .5 * (P[0]**2 + P[1]**2 - Q[0]**2 - Q[1]**2) / (P[0] -  Q[0])
  k2 = (P[1] - Q[1]) / (P[0] - P[1])
  b = .5 * (Q[0]**2 - 2 * Q[0] * k1 + Q[1]**2-R[0]**2+2*R[0]*k1-R[1]**2) / (R[0] * k2 - R[1]+Q[1]-Q[0] * k2)
  a = k1 - k2 * b
  r = ((P[0] - a)**2+(P[1] - b)**2)**1/2
  return 1/r






  






def timed_call(callback, calls_per_second, *args, **kw): # calls a function x number of times per second
    """
    Create an iterator which will call a function a set number
    of times per second.
    """
    time_time = time.time
    start = time_time()
    period = 1.0 / calls_per_second
    while True:
        if (time_time() - start) > period:
            start += period
            callback(*args, **kw)
        yield None 
        
timed_call(math)