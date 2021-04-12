#! /usr/bin/python

import rospy
import heapq

def dijkstra(graph,start):

	# print(graph)
	# for node in graph:
		# print(node)
	distances = {node: float('inf') for node in graph }
	
	print(distances)
	distances[start] = 0 
	queue = [] 
	heapq.heappush(queue, [distances[start],start])
	# print(queue)	

	while queue:
		current_distance, current_node = heapq.heappop(queue)
		print("current_distance:{0},current_node:{1}".format(current_distance,current_node))
		if distances[current_node] < current_distance:
			continue
		
		for adjacent, weight in graph[current_node].items():
			# print(current_node)
			print("adjacent:{0},weight:{1}".format(adjacent,weight))
			distance = current_distance + weight

			if distance < distances[adjacent]:
				distances[adjacent] = distance
				heapq.heappush(queue, [distances[adjacent],adjacent])
	
	return distances

mygraph = {
	'A': {'B': 8, 'C': 1, 'D': 2},
	'B': {},
	'C': {'B': 5, 'D':2},
	'D': {'E': 3, 'F':5},
	'E': {'F': 1},
	'F': {'A': 5}
}

print(dijkstra(mygraph, 'A'))

'''
n , m = 7 , 7 
g=[[""]*(m+1) for _ in range(n+1)]
for i in range(1,n+1) :
	for j in range(1,m+1):
		if i==j:
			g[i][j]=0
		if g[i][j] is "":
			g[i][j]=input(f"g{i}{j}= ")
			g[j][i]=g[i][j]

for i in range(1, n+1) : 
	for j in range(1,m+1) : 
		print(g[i][j],end='\t')
	print()

distance=[]
visit=[]
'''