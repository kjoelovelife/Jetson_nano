#!/usr/bin/env python
import rospy
import unittest
import rostest
import tf

class DuckietownDescriptionTesterNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('duckietown_description_tester_node', anonymous=False)

        # Setup the tf listener
        self.tfl = tf.TransformListener()

    def test_robot_description_param(self):
        param_exists = rospy.get_param("robot_description", False) is not False
        self.assertTrue(param_exists)

    def test_world_frame_exists(self):
        self.setup()

        # Wait up to 5 seconds for the transform to become available
        self.tfl.waitForTransform("world", "tile_0_0", rospy.Time(), rospy.Duration(5))
        world_exists = self.tfl.frameExists("world")
        self.assertTrue(world_exists)

    def test_edge_tile_frame_correct(self):
        self.setup()

        # Wait up to 5 seconds for the transform to become available
        self.tfl.waitForTransform("world", "tile_1_1", rospy.Time(), rospy.Duration(5))
        transform_exists = self.tfl.canTransform("world", "tile_1_1", rospy.Time())
        self.assertTrue(transform_exists)

        # Get the param for tile_width
        tile_width = rospy.get_param("~tile_width")
        trans = self.tfl.lookupTransform("world", "tile_1_1", rospy.Time())
        position = trans[0]
        self.assertAlmostEqual(position[0], tile_width)
        self.assertAlmostEqual(position[1], tile_width)
        self.assertAlmostEqual(position[2], 0)


if __name__ == '__main__':
    rostest.rosrun('duckietown_description', 'duckietown_description_tester_node', DuckietownDescriptionTesterNode)
