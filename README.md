# rospub: an interactive message publisher.


Publish messages directly from the terminal to a topic.

Usage: `rospub <topic> <message type>`

where:
  - topic: ROS topic messages should be published to
  - message: ROS message type (e.g., std_msgs.msg.String, geometry_msgs.msg.Point).


We supports simple types:

    ./rospub /foo std_msgs.msg.Int32
    >>> 42
    (publishes an Int32(42))

and more complex aggregate types:

    ./rospub /foo geometry_msgs.msg.Point
    >>> 1, 2, 3
    (publishes a Point(x=1, y=2, z=3))

