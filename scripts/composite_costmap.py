#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

LANE_COSTMAP = "/lane_costmap/costmap/costmap"
OBSTACLE_COSTMAP = "/obstacle_costmap/costmap/costmap"
COMPOSITE_COSTMAP = "/composite_costmap/costmap/costmap"


class CompositeCostmap:
    def __init__(self):
        rospy.init_node("composite_costmap")
        self.rate = rospy.Rate(5)

        self.lane_flag = False
        self.obstacle_flag = False

        self.composite_map = OccupancyGrid()
        self.lane_map = OccupancyGrid()
        self.obstacle_map = OccupancyGrid()

        self.lane_costmap_sub = rospy.Subscriber(LANE_COSTMAP, OccupancyGrid,
                                                  self.lane_costmap_callback)

        self.obstacle_costmap_sub = rospy.Subscriber(OBSTACLE_COSTMAP,
                                                     OccupancyGrid,
                                                     self.obstacle_costmap_callback)

        self.composite_pub = rospy.Publisher(COMPOSITE_COSTMAP, OccupancyGrid,
                                             queue_size=10)

    def lane_costmap_callback(self, msg):
        self.lane_flag = True
        self.lane_map = msg

    def obstacle_costmap_callback(self, msg):
        self.obstacle_flag = True
        self.obstacle_map = msg

    def calculate_composite(self):
        if not self.lane_flag:
            return False
        if not self.obstacle_flag:
            return False

        self.composite_map.header = self.lane_map.header
        self.composite_map.info = self.lane_map.info

        data = list(self.lane_map.data)

        # Change to numpy for efficiency
        for idx, obstacle_cost in enumerate(self.obstacle_map.data):
            data[idx] = max(data[idx], obstacle_cost)

        self.composite_map.data = tuple(data)

        return True

    def run(self):
        while not rospy.is_shutdown():
            if self.calculate_composite():
                self.composite_pub.publish(self.composite_map)
                # print("Composite generated and published")


if __name__ == '__main__':
    node = CompositeCostmap()
    node.run()
