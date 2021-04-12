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


load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity_prev'))
mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes = node_set.nodes
links = link_set.lines

# for i,idx in enumerate(node_set.nodes):

# 	print(i, nodes[idx].point[0])

# for i,idx in enumerate(links) :
# 	print(i,links[idx].points) 

# print("# of nodes: ",node_set.nodes)
# print("# of links: ", len(link_set.lines))


class cloud:

	def __init__(self):

		self.point_pub = rospy.Publisher(
			"/scan_",
			PointCloud,
			queue_size=5
		)
		
		self.link_pub = rospy.Publisher(
			"/link_",
			PointCloud,
			queue_size = 5
		)
		
		# mygraph = self.draw_mg()
		# print(dijkstra(mygraph,"A119BS010695"))


		# self.point_pub.publish(self.Points(nodes))
		# print(self.Points(nodes))
		# self.Points(nodes)

	def Points(self, ns_):
		_pc= PointCloud()
		_pc.header.frame_id = "map"
				
		for idx in ns_:
			_p = Point32()
			_p.x = ns_[idx].point[0]
			_p.y = ns_[idx].point[1]
			_p.z = ns_[idx].point[2]
			_pc.points.append(_p)
			# print(_pc)
		self.point_pub.publish(_pc)

	def links(self,links_):

		_pc= PointCloud()
		_pc.header.frame_id = "map"
		# num=0
		for idx in links_ :
			# num+=1
			for _point in links_[idx].points:
				_p = Point32()
				_p.x = _point[0]
				_p.y = _point[1]
				_p.z = _point[2]
				_pc.points.append(_p)
			# print(num)
		self.link_pub.publish(_pc)

	def draw_mg(self):	
		mygraph = { idx : self.node_weight(idx,nodes,links) for idx in nodes }
		return mygraph
		# print(mygraph)
		# print("end")

	def node_weight(self,_idx,_nodes,_links):
		idx_list=[]
		graph2={}
		for i in _nodes[_idx].get_to_links_idx_list():
			idx_list.append(i)
			for idxl in range(len(idx_list)):
				end_node1 = links[idx_list[idxl]].to_node
				end_node = end_node1.idx
				weight=len(links[idx_list[idxl]].points)
				graph1 = { end_node : weight}
				graph2.update(graph1)
			# print(graph)
		idx_list=[]
		return graph2

	def dijkstra(graph,start):

		distances = {node: float('inf') for node in graph}

		distances[start] = 0
		queue = []
		heapq.heappush(queue, [distances[start],start])

		while queue:
			currnet_distance, current_node = heapq.heappop(queue)
			if distances[current_node] < currnet_distance:
				continue
			for adjacent , weight in graph[current_node].items():
				distance = current_distance + weight 

				if distance < distances[adjacent]:
					distances[adjacent] = distance
					heapq.heappush(queue,[distances[adjacent],adjacent])
		return distances

if __name__ == "__main__":

	rospy.init_node("point_link_pub",anonymous = False)
	pub = cloud()
	
	# rospy.spin()
	print(pub.dijkstra(pub.draw_mg(),"A119BS010695"))
	# pub.draw_mg()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.Points(nodes)
		pub.links(links)
		rate.sleep()
	

	


