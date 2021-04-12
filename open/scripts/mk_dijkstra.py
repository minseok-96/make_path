#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class Dijkstra:

	def __init__(self):

		self.point_sub = rospy.Subscriber(
			"/scan_",
			PointCloud,
			callback= self.mk_matrix1
		)
		# self.point_sub = rospy.Subscriber(
		# 	"/link_",
		# 	PointCloud,
		# 	callback=self.mk_matrix2
		# )
	def mk_matrix1(self,mk_point):
		print(len(mk_point.points))

		
	# def mk_matrix2(self,mk_link):




if __name__ == "__main__":

	rospy.init_node("dijkstra",anonymous=False)

	algo = Dijkstra()

	rospy.spin()

