#! /usr/bin/python

import heapq

queue = []

heapq.heappush(queue, [2,'A'])
heapq.heappush(queue, [5,'B'])
heapq.heappush(queue, [1,'C'])
heapq.heappush(queue, [7,'D'])
print(queue)