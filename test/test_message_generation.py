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


if __name__ == '__main__':
    unittest.main()
