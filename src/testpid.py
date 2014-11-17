#!/usr/bin/env python
import rospid

x_pid = rospid.rospid(0.15,0.0,0.22,'x')
u = x_pid.update(0.0,0.1,1.0)
print u 
