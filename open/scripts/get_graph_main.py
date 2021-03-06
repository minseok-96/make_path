#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from sensor_msgs.msg import PointCloud
import heapq

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

'''
# distances = {node: float('inf') for node in graph }
idx_list=[]
# for idx in nodes:
	# print("idx:{}".format(idx))
for i in nodes["A119BS010223"].get_to_links_idx_list():
	idx_list.append(i)
	# print(idx_list)
print(idx_list)
link_=links["A219BS010418-A219BS010417"].to_node
node_=link_.idx
print(node_)
print(links[idx_list[0]].idx)
idx_list=[]
'''

def node_weight(_idx,_nodes,_links):
	idx_list=[]
	graph2={}
	for i in _nodes[_idx].get_to_links_idx_list():
		# print(i)
		idx_list.append(i)
		for idxl in range(len(idx_list)):
			end_node1=links[idx_list[idxl]].to_node
			end_node=end_node1.idx
			weight=len(links[idx_list[idxl]].points)
			graph1 = {end_node : weight}
			graph2.update(graph1)
	idx_list=[]
	# print(graph2)
	# print("end")
	return (graph2)		

def draw_mg(_nodes ,_links):	
	mygraph = { idx : node_weight(idx,_nodes,_links) for idx in _nodes }
	return mygraph

def dijkstra(graph,start,end):

	distances = {node: [float('inf'),start] for node in graph}

	distances[start] = [0,start]
	queue = []
	heapq.heappush(queue, [distances[start][0],start])

	while queue:
		current_distance, current_node = heapq.heappop(queue)
		if distances[current_node][0] < current_distance:
			continue
		for adjacent , weight in graph[current_node].items():
			distance = current_distance + weight 

			if distance < distances[adjacent][0]:
				distances[adjacent] = [distance,current_node]
				heapq.heappush(queue,[distances[adjacent][0],adjacent])
	path = end
	path_output = end + "->"
	while distances[path][1] != start:
		path_output += distances[path][1] + "->"
		path = distances[path][1]
	path_output += start
	print(path_output)
	return distances

# print(draw_mg(nodes,links))
print(dijkstra(draw_mg(nodes,links),"A119BS010695","A119BS010327"))


# print( nodes["A119BS010223"].get_to_links_idx_list())


# mygraph ={ node : for idx in nodes}
# for idx in nodes:
	# print(id(nodes[idx].to_links))
	# print(idx)
# 	print(nodes[idx].get_to_links())
# print(nodes["A119BS010225"])
# print (nodes["A119BS010225"].print_all_related_nodes_and_links())

# print (len(nodes["A119BS010223"].get_to_nodes()))
# print (nodes["A119BS010223"].get_to_nodes())
# nodes["A119BS010713"].print_all_related_nodes_and_links()
# nodes["A119BS010712"].print_all_related_nodes_and_links()
# nodes["A119BS010714"].print_all_related_nodes_and_links()
# print(type(nodes["A119BS010223"].to_links))
# for i in nodes["A119BS010223"].get_to_links_idx_list():
	# print(i)
# print(len(links["A219BS010418-A219BS010419"]))
"weight"
# print(len(links["A219BS010418"].points))

# for index, idx in enumerate(nodes):

# 	print(index,idx)















# for idx in links :
	# print(links[idx].points)
	# print(idx)
	# for _point in links[idx].points:
	# 	print(_point)
	# # print(i,links[idx].points)

# def dijkstra(graph,start):

#  	for idx in nodes: