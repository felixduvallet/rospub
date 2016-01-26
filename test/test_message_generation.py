import unittest

import imp
import os

# Do some trickery to import the rospub module (without the .py extension) by
# using the imp module and specifying the path of the rospub module directly.
dirname = os.path.dirname(os.path.realpath(__file__))  # path of test module
imp.load_source('rospub', dirname+'/../rospub')
import rospub

import std_msgs.msg
import geometry_msgs.msg


class TestCase(unittest.TestCase):

    def test_string(self):
        msg_type = 'std_msgs.msg.String'
        content = 'hello world'

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, std_msgs.msg.String)
        self.assertIsInstance(msg.data, basestring)
        self.assertEqual(content, msg.data)

    def test_string_of_int(self):
        msg_type = 'std_msgs.msg.String'
        content = '123'

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, std_msgs.msg.String)
        self.assertIsInstance(msg.data, basestring)
        self.assertEqual(content, msg.data)

    def test_int32(self):
        msg_type = 'std_msgs.msg.Int32'
        content = '42'  # raw_input will always return a string.

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, std_msgs.msg.Int32)
        self.assertIsInstance(msg.data, int)
        self.assertEqual(int(content), msg.data)

    def test_float32(self):
        msg_type = 'std_msgs.msg.Float32'
        content = '3.14159'  # raw_input will always return a string.

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, std_msgs.msg.Float32)
        self.assertIsInstance(msg.data, float)
        self.assertEqual(float(content), msg.data)

    def test_bool(self):
        msg_type = 'std_msgs.msg.Bool'
        content = 'True'  # raw_input will always return a string.

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, std_msgs.msg.Bool)
        self.assertIsInstance(msg.data, bool)
        self.assertEqual(True, msg.data)
        # NB: creating the message does *not* cast the data directly to a bool;
        # this appears to occur during message publishing.

    def test_point(self):
        msg_type = 'geometry_msgs.msg.Point'
        content = '3.1, 4.1, 2.1'  # raw_input will always return a string.

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, geometry_msgs.msg.Point)
        self.assertIsInstance(msg.x, float)
        self.assertIsInstance(msg.y, float)
        self.assertIsInstance(msg.z, float)
        self.assertEqual(msg.x, 3.1)
        self.assertEqual(msg.y, 4.1)
        self.assertEqual(msg.z, 2.1)

    def test_too_little_information(self):
        msg_type = 'geometry_msgs.msg.Point'
        content = '42'  # This is *not* enough information for a point

        msg = rospub.make_message(msg_type, content)
        self.assertIsNone(msg)

    def test_too_much_information(self):
        msg_type = 'geometry_msgs.msg.Point'
        content = '42, 41, 40, 39'  # too much information.

        msg = rospub.make_message(msg_type, content)
        self.assertIsNone(msg)


if __name__ == '__main__':
    unittest.main()
