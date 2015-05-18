# Introduction #

This package provides a python library with a standard PID controller class.  Each controller is externally tunable via ROS topics.  A GUI tuning node is provided.

Note that the PID controller does not subscribe or publish directly to topics for control - those actions are done in code by calling the PID object's `update` method.  This is intended to enable a wide variety of input and output processing, e.g. multiple loop closures in a single callback.

# PID Object Usage #

Here's an example of the PID constructor:
```
pid_obj = rospidlib.Rospid(0.15,0.0,0.22,'~my_pid')
```

This will make a new PID object with default gains of 0.15 proportional, 0 integral and 0.22 derivative.  "~my\_pid" indicates that the tuning topics will be made available in the private namespace of the node, under the subname "my\_pid".  This feature enables you to have multiple PIDs existing in the same node.  E.g. if my node is named "/drone1/control" and I set the PID namespace to be "pitch", the topics and parameters will all be in "/drone1/control/pitch".

## Parameters ##

You can overwrite the hardcoded initial parameters by setting parameters "init\_gains/kp" "init\_gains/ki" or "init\_gains/kd" in the controller namespace.  E.g. in the example above, set "/drone1/control/pitch/init\_gains/kp" to change the default gain settings. **These parameters are only read at startup.**  To avoid delays in the loops, online tuning is done via topics.  See the example below for how to include these settings in a launch file.

## Subscribed Topics ##

The controller will subscribe to "Float32" type topics "tune\_gains/kp" "tune\_gains/ki" and "tune\_gains/kd" in its namespace. E.g. in the example above you can tune by publishing to "/drone1/control/pitch/tune\_gains/kp" and so on.  You can do this using `rostopic pub` or using the "tune\_gui" node provided.

## Utility Functions ##

` output = rospidlib.saturate(input, limit) `

This function will clamp the output between -limit and +limit.  A ROS warning is produced if the limit is active.

` output = rospidlib.saturate2(input, lolimit, hilimit) `

This function will clamp the output between lolimit and hilimit.  A ROS warning is produced if the limit is active.  Another warning occurs if `lolimit` is the same or higher than `hilimit`.

# Tuning GUI #

The tune\_gui node enables simple online tuning of each PID object.  Whenever a fresh gain number is entered in one of the three gain boxes, an update message is sent to the controller.  Updates are sent to the "tune\_gains/kx" topic in the namespace where the tune\_gui node was started.  To assign to a specific PID, push the tune\_gui node to its namespace, e.g. for the example given above:
```
ROS_NAMESPACE = /drone1/control/pitch rosrun rospid tune_gui
```
The same could be accomplished with three remaps.  The tune\_gui node will attempt to load initial gain settings from the "init\_gains" parameters, or default to zero if those cannot be found.

# Example #

## Demo Node ##

The "circle\_turtle" node provides an example of the usage of the Rospid class.  This node listens for pose messages from a turtlesim simulator.  The PID controller sets a constant speed and steers the turtle to maintain 2.5 units distance from the point (5,7).  If all goes well, the turtle will end up in a circle as demanded.

## Launch File ##

Just to illustrate namespace handling, the whole demonstration is pushed to namespace `turtle_control`.

```
<launch>

  <group ns="turtle_control">
    
    <node pkg="rospid" name="control" type="circle_turtle.py">
      <param name="steering/init_gains/kp" type="double" value="0.3" />
      <param name="steering/init_gains/ki" type="double" value="0.0" />
      <param name="steering/init_gains/kd" type="double" value="0.3" />
    </node>

    <node pkg="rospid" name="tuning" type="tune_gui.py" ns="control/steering" />

    <node pkg="turtlesim" name="turtlesim" type="turtlesim_node" />
    
  </group>

</launch>
```



It is used in the 'testpid.launch' example.

# Details #

Add your content here.  Format your content with:
  * Text in **bold** or _italic_
  * Headings, paragraphs, and lists
  * Automatic links to other wiki pages