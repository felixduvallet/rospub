import unittest

import imp
import os

# Do some trickery to import the rospub module (without the .py extension) by
# using the imp module and specifying the path of the rospub module directly.
dirname = os.path.dirname(os.path.realpath(__file__))  # path of test module
imp.load_source('rospub', dirname+'/../rospub')
import rospub


class TestCase(unittest.TestCase):
    def test_something(self):
        #ret = rospub.make_message('asdf', 'fff')
        #self.assertIsNotNone(ret)
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
