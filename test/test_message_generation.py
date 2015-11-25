import unittest

import imp
import os

# Do some trickery to import the rospub module (without the .py extension) by
# using the imp module and specifying the path of the rospub module directly.
dirname = os.path.dirname(os.path.realpath(__file__))  # path of test module
imp.load_source('rospub', dirname+'/../rospub')
import rospub

import std_msgs


class TestCase(unittest.TestCase):

    def test_string(self):
        msg_type = 'std_msgs.msg.String'
        content = 'hello world'

        msg = rospub.make_message(msg_type, content)
        self.assertIsNotNone(msg)

        self.assertIsInstance(msg, std_msgs.msg.String)
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


if __name__ == '__main__':
    unittest.main()
