import rospy

class rospid:

  def __init__(self, kp, ki, kd, namespace):
    # constructor
    # namespace is location for tuning parameters

    # store default gains
    self.kp = kp
    self.ki = ki
    self.kd = kd

    # stores for last input values for differentiating
    self.last_t = 0
    self.last_y = 0
    # flag for whether or not stores have been initialized
    self.has_run = False

    # integrator
    self.integ = 0
    self.last_e = 0

  def update(self, y, r, t):
    # return new output given new inputs
    # y is measurement; r is reference; t is time

    # start with proportional gain
    u = self.kp*(r - y)
    
    # only use the I and D parts if valid stored data
    if self.has_run:
      # find time since last update
      delta_t = t - self.last_t

      # add to integrator using trapezium rule
      self.integ = self.integ + self.ki*0.5*delta_t*(self.last_e + r-y)

      # add integral term to control
      u = u + self.integ

      # find derivatives using finite difference
      dydt = (y - self.last_y)/delta_t

      # add to control - note negative
      u = u - self.kd*dydt

      # stores for future use
      self.last_e = r-y
      self.last_y = y
      self.last_t = t
      self.has_run = True




