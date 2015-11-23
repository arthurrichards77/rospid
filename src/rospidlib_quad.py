import rospy
import roslib
import math
roslib.load_manifest('rospid')
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped, Point, Twist
from tf.transformations import euler_from_quaternion

def saturate2(inp,lolimit,hilimit):
  # limit quantity to [lolimit,hilimit]
  if lolimit >= hilimit:
    rospy.logwarn('No room between limits [%f, %f]', lolimit, hilimit)
  out = inp
  if inp>hilimit:
    out=hilimit
    rospy.logwarn('Clamping %f at upper limit %f', inp, hilimit)
  elif inp<lolimit:
    out = lolimit
    rospy.logwarn('Clamping %f at lower limit %f', inp, lolimit)
  return out

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = saturate2(inp, -limit, limit)
  return out

class RospidQuad:

  def __init__(self, namespace):
    # constructor
    # namespace is location for tuning parameters

    # store own namespace
    self.namespace = rospy.resolve_name(namespace)
    rospy.loginfo('Setting up new PID controller in %s', self.namespace)

    # Position feedback gains
    self.KP = Twist()
    self.KI = Twist()
    self.KD = Twist()

    # QAV 250 gains as default, or load from parameters
    self.KP.linear.x = rospy.get_param(self.namespace + "/pitch/init_gains/kp",0.22)
    self.KP.linear.y = rospy.get_param(self.namespace + "/roll/init_gains/kp",0.22)
    self.KP.linear.z = rospy.get_param(self.namespace + "/thrust/init_gains/kp",0.4)
    self.KP.angular.z = rospy.get_param(self.namespace + "/yaw/init_gains/kp",0.5)

    self.KI.linear.x = rospy.get_param(self.namespace + "/pitch/init_gains/ki",0.04)
    self.KI.linear.y = rospy.get_param(self.namespace + "/roll/init_gains/ki",0.04)
    self.KI.linear.z = rospy.get_param(self.namespace + "/thrust/init_gains/ki",0.06)
    self.KI.angular.z = rospy.get_param(self.namespace + "/yaw/init_gains/ki",0)

    self.KD.linear.x = rospy.get_param(self.namespace + "/pitch/init_gains/kd",0.28)
    self.KD.linear.y = rospy.get_param(self.namespace + "/roll/init_gains/kd",0.28)
    self.KD.linear.z = rospy.get_param(self.namespace + "/thrust/init_gains/kd",0.36)
    self.KD.angular.z = rospy.get_param(self.namespace + "/yaw/init_gains/kd",0)

    rospy.logwarn('Roll gains:   [P, I, D] = [%5.2f, %5.2f, %5.2f]', self.KP.linear.x, self.KI.linear.x, self.KD.linear.x)
    rospy.logwarn('Pitch gains:  [P, I, D] = [%5.2f, %5.2f, %5.2f]', self.KP.linear.y, self.KI.linear.y, self.KD.linear.y)
    rospy.logwarn('Thrust gains: [P, I, D] = [%5.2f, %5.2f, %5.2f]', self.KP.linear.z, self.KI.linear.z, self.KD.linear.z)
    rospy.logwarn('Yaw gains:    [P, I, D] = [%5.2f, %5.2f, %5.2f]', self.KP.angular.z, self.KI.angular.z, self.KD.angular.z)

    # Pick up control reversals
    pitch_rev = rospy.get_param(self.namespace + "/pitch/reverse",0)
    roll_rev = rospy.get_param(self.namespace + "/roll/reverse",0)
    thrust_rev = rospy.get_param(self.namespace + "/thrust/reverse",0)
    yaw_rev = rospy.get_param(self.namespace + "/yaw/reverse",0)
    self.reverse = [pitch_rev, roll_rev, thrust_rev, yaw_rev]
    rospy.logwarn('Reversals: [Pitch, Roll, Thrust, Yaw] = [%i, %i, %i, %i]', self.reverse[0], self.reverse[1], self.reverse[2], self.reverse[3])
    for ax in range(0,len(self.reverse)):
      if self.reverse[ax] == 1:
        self.reverse[ax] = -1
      else:
        self.reverse[ax] = 1

    # stores for last input values for differentiating
    self.last_t = 0.0
    self.last_y = TransformStamped()
    # flag for whether or not stores have been initialized
    self.has_run = False

    # Position error, local and global frames
    self.e_local = Point()
    self.e_global = Point()

    # integrator
    self.integ = Point()
    self.last_e = Point()
    # option to disable integral action
    self.freeze_integrator_flag = False

    # create gain update subscribers
    self.sub_kp = rospy.Subscriber(self.namespace + "/tune_gains/kp", Float32, self.tune_kp_callback)
    self.sub_ki = rospy.Subscriber(self.namespace + "/tune_gains/ki", Float32, self.tune_ki_callback)
    self.sub_kd = rospy.Subscriber(self.namespace + "/tune_gains/kd", Float32, self.tune_kd_callback)
    
    # closing message
    rospy.loginfo('PID ready in %s', self.namespace)

  def update(self, y, r, t):
    # return new output given new inputs
    # y is measurement (TransformStamped); r is reference (TransformStamped); t is time

    # Grab measured yaw value
    quaternion = (
      y.transform.rotation.x,
      y.transform.rotation.y,
      y.transform.rotation.z,
      y.transform.rotation.w)
    y_euler = euler_from_quaternion(quaternion)
    y_yaw = y_euler[2]

    # Grab reference yaw value
    quaternion = (
      r.transform.rotation.x,
      r.transform.rotation.y,
      r.transform.rotation.z,
      r.transform.rotation.w)
    r_euler = euler_from_quaternion(quaternion)
    r_yaw = r_euler[2]

    # Calculate the global error in position
    self.e_global.x = r.transform.translation.x - y.transform.translation.x
    self.e_global.y = r.transform.translation.y - y.transform.translation.y
    self.e_global.z = r.transform.translation.z - y.transform.translation.z

    # Convert to local coordinates
    self.e_local.x = self.e_global.x*math.cos(y_yaw) + self.e_global.y*math.sin(y_yaw)
    self.e_local.y = self.e_global.x*math.sin(y_yaw) - self.e_global.y*math.cos(y_yaw)
    self.e_local.z = self.e_global.z
    e_yaw = r_yaw - y_yaw
    if e_yaw > math.pi:
        e_yaw = e_yaw-2*math.pi
    elif e_yaw < -math.pi:
        e_yaw = e_yaw+2*math.pi

    # start with proportional gain
    u = Twist()
    u.linear.x = self.KP.linear.x*self.e_local.x*self.reverse[0]
    u.linear.y = self.KP.linear.y*self.e_local.y*self.reverse[1]
    u.linear.z = self.KP.linear.z*self.e_local.z*self.reverse[2]
    u.angular.z = self.KP.angular.z*e_yaw*self.reverse[3]

    # only use the I and D parts if valid stored data
    if self.has_run == True:

      # find time since last update
      delta_t = t - self.last_t

      # print t, self.last_t, delta_t

      # optional integrator update
      if not self.freeze_integrator_flag:

        # add to integrator using trapezium rule
        self.integ.x = self.integ.x + self.KI.linear.x*0.5*delta_t*(self.last_e_local.x + self.e_local.x)
        self.integ.y = self.integ.y + self.KI.linear.y*0.5*delta_t*(self.last_e_local.y + self.e_local.y)
        self.integ.z = self.integ.z + self.KI.linear.z*0.5*delta_t*(self.last_e_local.z + self.e_local.z)

      # add integral term to control
      u.linear.x = u.linear.x + self.integ.x*self.reverse[0]
      u.linear.y = u.linear.y + self.integ.y*self.reverse[1]
      u.linear.z = u.linear.z + self.integ.z*self.reverse[2]

      # find derivatives using finite difference
      #dydt = (y - self.last_y)/delta_t
      dydt_global = Point()
      dydt_global.x = (y.transform.translation.x - self.last_y.transform.translation.x)/delta_t
      dydt_global.y = (y.transform.translation.y - self.last_y.transform.translation.y)/delta_t
      dydt_global.z = (y.transform.translation.z - self.last_y.transform.translation.z)/delta_t

      dydt_local = Point()
      dydt_local.x = dydt_global.x*math.cos(y_yaw) + dydt_global.y*math.sin(y_yaw)
      dydt_local.y = dydt_global.x*math.sin(y_yaw) - dydt_global.y*math.cos(y_yaw)
      dydt_local.z = dydt_global.z

      # add to control - note negative
      u.linear.x = u.linear.x - self.KD.linear.x*dydt_local.x*self.reverse[0]
      u.linear.y = u.linear.y - self.KD.linear.y*dydt_local.y*self.reverse[1]
      u.linear.z = u.linear.z - self.KD.linear.z*dydt_local.z*self.reverse[2]

    # stores for future use
    self.last_e_local = self.e_local
    self.last_y = y
    self.last_t = t
    self.has_run = True

    # return the new control value
    return u

  # callbacks for online tuning

  def tune_kp_callback(self, data):
    # get new gains from ROS parameters
    self.kp = data.data
    rospy.logwarn('PID %s updated kp: %f', self.namespace, self.kp)

  def tune_ki_callback(self, data):
    # get new gains from ROS parameters
    self.ki = data.data
    rospy.logwarn('PID %s updated ki: %f', self.namespace, self.ki)

  def tune_kd_callback(self, data):
    # get new gains from ROS parameters
    self.kd = data.data
    rospy.logwarn('PID %s updated kd: %f', self.namespace, self.kd)

  # utilities for manually resetting and freezing integrator

  def reset_integrators(self, new_value):
    # reset integrator values
    self.integ = new_value
    rospy.logwarn('PID %s reset integrators to %.2f, %.2f, %.2f', self.namespace, self.integ.x, self.integ.y, self.integ.z)

  def read_integrators(self):
    # read integrator values
    return self.integ

  def freeze_integrator(self):
    if not self.freeze_integrator_flag:
      rospy.logwarn('PID %s frozen integrators at %.2f, %.2f, %.2f', self.namespace, self.integ.x, self.integ.y, self.integ.z)
    self.freeze_integrator_flag = True

  def enable_integrator(self):
    if self.freeze_integrator_flag:
      rospy.logwarn('PID %s enabled integrators at %.2f, %.2f, %.2f', self.namespace, self.integ.x, self.integ.y, self.integ.z)
    self.freeze_integrator_flag = False

  def read_integrator(self):
    return(self.integ)
