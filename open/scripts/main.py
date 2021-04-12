#! /usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np


class pcl_processor:
    __slots__ = ['pcl_sub', 'pcl_pub', 'marker_pub']
    def __init__(self):
        # RPLiDAR의 /scan 데이터 받는 객체
        self.pcl_sub = rospy.Subscriber(
            "/scan",
            LaserScan,
            self.callback
        )
        # Clustering된 PointCloud를 보내는 객체
        self.pcl_pub = rospy.Publisher(
            "/clustered_pcl",
            PointCloud,
            queue_size=10
        )
        # Clustering 오브젝트를 표현하는 Marker를 보내는 객체
        self.marker_pub = rospy.Publisher(
            "/pclMarker",
            MarkerArray,
            queue_size=5
        )

    # X, Y 좌표 계산
    def calcAxis(self, _distance:float, _rad:float) -> [float, float]:
        _x = math.cos(_rad) * _distance
        _y = math.sin(_rad) * _distance
        return _x, _y

    # 두 점 사이의 거리 계산
    def calcDistance(self, _p1:list, _p2:list) -> float:
        _x1, _y1 = _p1
        _x2, _y2 = _p2

        _distance = math.sqrt(
            math.pow(_x2 - _x1, 2) + math.pow(_y2 - _y1, 2)
        )

        return _distance

    # Marker 초기화 및 생성
    def setMarker(self, _p:list, _id:int, _size:float) -> Marker:
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.ns = "position"
        marker.id = _id
        marker.lifetime = rospy.Duration.from_sec(0.1)

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = _p[0]
        marker.pose.position.y = _p[1]
        marker.pose.position.z = 0.01

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = _size
        marker.scale.y = _size
        marker.scale.z = _size

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

    # PointCloud로 변환
    def makePointCloud(self, _p) -> Point32:
        _point = Point32()
        _point.x = _p[0]
        _point.y = _p[1]
        _point.z = 0.1
        return _point

    # Clustering 함수
    def groupping(self, pcl:LaserScan) -> list:
        min_rad = pcl.angle_min
        rad_inc = pcl.angle_increment
        _dgroup = 0.005
        _dp = 0.005
        _index = 1

        temp = []   # ABD결과 적합한 Point 리스트
        cluster = []# 적합한 Point Group을 담는 리스트(리스트를 담는 리스트)

        for i in range(1, len(pcl.ranges)):
            if pcl.ranges[i - _index] == np.inf:
                continue
            elif pcl.ranges[i] == np.inf:
                _index += 1
                continue

            _p1 = self.calcAxis(
                pcl.ranges[i - _index],
                min_rad + (rad_inc * (i - _index))
            )
            _p2 = self.calcAxis(
                pcl.ranges[i],
                min_rad + (rad_inc * i)
            )

            # ABD (Adaptive Breakpoint Detection)
            _dist = self.calcDistance(_p1, _p2)
            _ridp = _dp * self.calcDistance(_p1, [0, 0])

            if _dist < (_dgroup + _ridp):
                temp.append(_p1)
            else:
                if len(temp) > 5: # Point의 갯수가 5개 초과만 오브젝트로 취급
                    cluster.append(temp)
                temp = []
            _index = 1

        _pcl = PointCloud()
        _n = 0

        for _ps in cluster:
            for i in _ps:
                _pcl.points.append(self.makePointCloud(i))

        _pcl.header.frame_id = "laser"
        self.pcl_pub.publish(_pcl)

        return cluster

    def callback(self, pcl:LaserScan):
        grouped_pcl = self.groupping(pcl)

        mkArray = MarkerArray()
        mkArray.markers = []

        _id = 0

        for points in grouped_pcl:
            # 시작점과 끝점
            _p0 = points[0]
            _p1 = points[-1]

            # 두 점 사이의 거리
            _tri_line = self.calcDistance(_p0, _p1) / 2.0

            # 오브젝트 중간점
            _pm = [
                (_p0[0] + _p1[0]) / 2.0,
                (_p0[1] + _p1[1]) / 2.0
            ]

            _mk = self.setMarker(_pm, _id, _tri_line)
            mkArray.markers.append(_mk)
            _id += 1

        self.marker_pub.publish(mkArray)


if __name__ == "__main__":
    rospy.init_node("pcl_processor", anonymous=False)
    pp = pcl_processor()
    rospy.spin()
        