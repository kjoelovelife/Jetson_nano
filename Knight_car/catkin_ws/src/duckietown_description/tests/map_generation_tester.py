#!/usr/bin/env python
import rospy
import unittest
import rostest
import tf
import os, os.path
import rospkg
import subprocess

class MapGenerationTester(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('map_generation_tester_node', anonymous=True)

        # Setup the tf listener
        self.tfl = tf.TransformListener()
        rp = rospkg.RosPack()
        map_name = rospy.get_param("~map_name")
        self.path = "{pkg_root}/urdf/{map_name}.urdf.xacro".format(pkg_root=rp.get_path("duckietown_description"), map_name=map_name)

        # Wait for the map generator  to finish saving the file
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while not os.path.isfile(self.path) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertTrue(os.path.isfile(self.path), "Test timed out while waiting for map file to be generated")
    #
    # def test_map_file_generates(self):
    #     self.setup()  # Setup the node


    def test_csv_to_transform(self):
        self.setup()

        # Launch the duckietown_description
        os.system("roslaunch duckietown_description duckietown_description_node.launch veh:=testbot gui:=false map_name:=test_map_tmp &")

        # Wait up to n seconds for the transform to become available
        try:
            self.tfl.waitForTransform("world", "tile_1_1", rospy.Time(), rospy.Duration(10))
        except:
            self.assertTrue(False, "Test timed out while waiting for transform.")

        transform_exists = self.tfl.canTransform("world", "tile_1_1", rospy.Time())
        self.assertTrue(transform_exists, "Couldn't find transform")

        # Get the param for tile_width
        tile_width = rospy.get_param("~tile_width")
        trans = self.tfl.lookupTransform("world", "tile_1_1", rospy.Time())
        position = trans[0]
        self.assertAlmostEqual(position[0], tile_width)
        self.assertAlmostEqual(position[1], tile_width)
        self.assertAlmostEqual(position[2], 0)

if __name__ == '__main__':
    rostest.rosrun('duckietown_description', 'map_generation_tester', MapGenerationTester)
