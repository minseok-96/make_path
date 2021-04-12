#! /usr/bin/python

import rospy
import heapq

def dijkstra(graph,start,end):

	# print(graph)
	# for node in graph:
		# print(node)
	distances = {node: [float('inf'),start] for node in graph }
	# insert [0,'A'] at key value
	distances[start] = [0,start] 
	# print(distances[start][1])
	queue = [] 
	heapq.heappush(queue, [distances[start][0],start])
	# print(queue)	

	while queue:
		print(queue)
		current_distance, current_node = heapq.heappop(queue)
		print("current_distance:{0},current_node:{1}".format(current_distance,current_node))
		# print("end")
		# print(distances[current_node][0])
		print(distances)
		if distances[current_node][0] < current_distance:
			print(distances[current_node][0])
			continue
		# extract key, value at graph
		for adjacent, weight in graph[current_node].items():
			# print(current_node)
			# print("adjacent:{0},weight:{1}".format(adjacent,weight))
			distance = current_distance + weight

			if distance < distances[adjacent][0]:
				# print(distances[adjacent])
				distances[adjacent] = [distance,current_node]
				# print(distances)
				# print("end")
				heapq.heappush(queue, [distances[adjacent][0],adjacent])
	path = end
	path_output = end + '->'
	while distances[path][1] != start:
		path_output += distances[path][1] + "->"
		path = distances[path][1]
	path_output += start
	print(path_output)
	return distances

mygraph = {
	'A': {'B': 8, 'C': 1, 'D': 2},
	'B': {},
	'C': {'B': 5, 'D':2},
	'D': {'E': 3, 'F':5},
	'E': {'F': 1},
	'F': {'A': 5}
}

print(dijkstra(mygraph, 'A','F'))

