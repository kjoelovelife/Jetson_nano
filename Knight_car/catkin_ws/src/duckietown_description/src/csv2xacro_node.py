#!/usr/bin/env python
import rospy
import rospkg
from duckietown_description import Csv2Xacro

'''
Map Parser Node
Author: Amado Antonini
Inputs:
    tile_map_csv    must be absolute path to file
    tag_map_csv     must be absolute path to file
    map_name        only name of file
Outputs:
    <map_name>.urdf.xacro map file
'''

class Csv2xacroNode(object):
    def __init__(self):
        self.node_name = 'csv2xacro_node'
        self.package_path = rospkg.RosPack().get_path('duckietown_description')

        self.tile_map_csv = self.setupParam("~tile_map_csv", self.package_path + '/tiles_226.csv')
        self.tag_map_csv = self.setupParam("~tag_map_csv", self.package_path + '/tags_default.csv')
        self.map_name = self.package_path + '/urdf/' + self.setupParam("~output_map_name", 'default_map') + '.urdf.xacro'
        self.tile_width = self.setupParam("~tile_width", 0.595)
        self.tag_offset = self.setupParam("~tag_offset", 0.125)
        self.tag_curb = self.setupParam("~tag_curb", 0.035)

        rospy.loginfo("[%s] has started", self.node_name)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def createXacroMap(self):
        writer = Csv2Xacro.Csv2Xacro(self.tile_map_csv, self.tag_map_csv, self.map_name, self.tile_width, self.tag_offset, self.tag_curb)
        writer.writeXacro()


if __name__ == '__main__':
    rospy.init_node('csv2xacro_node', anonymous=False)
    csv2xacro_node = Csv2xacroNode()
    csv2xacro_node.createXacroMap()
    rospy.loginfo("[%s] has finished", csv2xacro_node.node_name)