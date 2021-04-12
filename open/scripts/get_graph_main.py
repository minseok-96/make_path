#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity_prev'))
mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes = node_set.nodes
links = link_set.lines

print("# of nodes: ",len(node_set.nodes))
print("# of links: ", len(link_set.lines))

# for idx in nodes:
# 	print(idx)
# 	print(nodes[idx].get_to_links())
	
# print (nodes["A119BS010225"].get_to_links())
# print (nodes["A119BS010225"].print_all_related_nodes_and_links())

print (len(nodes["A119BS010223"].get_to_links()))
print (nodes["A119BS010223"].get_to_links())

# for idx in links :
	# print(links[idx].points)
	# print(idx)
	# for _point in links[idx].points:
	# 	print(_point)
	# # print(i,links[idx].points)

# def dijkstra(graph,start):

#  	for idx in nodes:



