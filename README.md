# rospub: an interactive message publisher.


Publish messages directly from the terminal to a topic.

Usage: `rospub <topic> <message type>`

where:
  - topic: ROS topic messages should be published to
  - message: ROS message type (e.g., std_msgs.msg.String).





TODO: use genpy
```
In [55]: g = geometry_msgs.msg.Point()

In [56]: d = dict(x=3, y=5, z=4)

In [57]: genpy.message.fill_message_args(g, d)

In [58]: g
Out[58]:
x: 3
y: 5
z: 4
```