import rospy

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

def saturate2(inp,lolimit,hilimit):
  # limit quantity to [lolimit,hilimit]
  out = inp
  if inp>hilimit:
    out=hilimit
  elif inp<lolimit:
    out = lolimit
  return out

class rospid:

  def __init__(self, kp, ki, kd, namespace):
    # constructor
    # namespace is location for tuning parameters

    # store default gains
    self.kp = kp
    self.ki = ki
    self.kd = kd

    # stores for last input values for differentiating
    self.last_t = 0.0
    self.last_y = 0.0
    # flag for whether or not stores have been initialized
    self.has_run = False

    # integrator
    self.integ = 0.0
    self.last_e = 0.0
    # option to disable integral action
    self.freeze_integrator_flag = False

    # store own namespace
    self.namespace = namespace

  def update(self, y, r, t):
    # return new output given new inputs
    # y is measurement; r is reference; t is time
    
    # start with proportional gain
    u = self.kp*(r - y)

    # only use the I and D parts if valid stored data
    if self.has_run == True:

      # find time since last update
      delta_t = t - self.last_t

      # print t, self.last_t, delta_t

      # optional integrator update
      if not self.freeze_integrator_flag:

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

    # return the new control value
    return u

  def refreshgains(self):
    # get new gains from ROS parameters
    # not implemented yet
    pass

  def resetintegrator(self, new_value):
    # reset integrator value
    self.integ = new_value

  def freeze_integrator(self):
    self.freeze_integrator_flag = True

  def enable_integrator(self):
    self.freeze_integrator_flag = False

  def read_integrator(self):
    return(self.integ)
