# Main and helper function

from mmap import MAP_SHARED
from tkinter import N
from PIL import Image
import numpy as np
from RRT import RRT
import rospy
import matplotlib.pyplot as plt
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import rospy
from basic_search import bfs, dijkstra, astar
import sys
from time import time

class Global_Planner:
        
    def rrt_srv_callback(self, req):
        global t0 
        global t1
        global timex
        print("Starting Planning. Start: ", req.start.pose.position.x, " ", req.start.pose.position.y, " Goal: ", req.goal.pose.position.x, " ", req.goal.pose.position.y)
        start = (int(req.start.pose.position.x), int(req.start.pose.position.y))
        goal  = (int(req.goal.pose.position.x), int(req.goal.pose.position.y))
        #chose algorithm based on command line output
        if self.algo == 'astar':
            t0= time()
            print('launching ASTAR ')
            plan_ = astar(self.map_array, start, goal, self.plot_on)
            t1=time()
            timex=t1-t0
            print("Time taken to find path",timex)
        elif self.algo == 'informed_rrt':
            t0= time()
            print('launching informed rrt star ')
            RRT_planner = RRT(self.map_array, start, goal, self.plot_on)
            plan_ = RRT_planner.informed_RRT_star(n_pts=2000)
            t1=time()
            timex=t1-t0
            print("Time taken to find path",timex)
        elif self.algo == 'rrt':
            t0= time()
            print('launching rrt ')
            RRT_planner = RRT(self.map_array, start, goal, self.plot_on)
            plan_ = RRT_planner.RRT(n_pts=5000)
            t1=time()
            timex=t1-t0
            print("Time taken to find path",timex)
        elif self.algo == 'rrt_star':
            t0= time()
            print('launching rrt_star ')
            RRT_planner = RRT(self.map_array, start, goal, self.plot_on)
            plan_ = RRT_planner.RRT_star(n_pts=5000)
            t1=time()
            timex=t1-t0
            print("Time taken to find path",timex)
        elif self.algo == 'dijkstra':
            t0= time()
            print('launching dijkstra ')
            plan_ = dijkstra(self.map_array, start, goal, self.plot_on)
            t1=time()
            timex=t1-t0
            print("Time taken to find path",timex)
        elif self.algo == 'bfs':
            t0= time()
            print('launching bfs ')
            plan_ = bfs(self.map_array, start, goal, self.plot_on)
            t1=time()
            timex=t1-t0
            print("Time taken to find path",timex)
        # elif self.algo == 'dfs':
        #     print('launching dfs')
        #     plan_ = dfs(self.map_array, start, goal, self.plot_on)
        else:
            print("Invalid usage! example usage: python main.py <algorithm>")
            print("choices of algorithms: astar, informed_rrt, rrt, rrt_star, dijkstra, bfs, dfs")

        #plan_ = astar(self.map_array, start, goal)
        plan = Path()
        if (self.algo == 'informed_rrt' or self.algo=='rrt' or self.algo == 'rrt_star'):
            for node in plan_:
                node_pose = PoseStamped()
                node_pose.pose.position.x = node[0]
                node_pose.pose.position.y = node[1]
                plan.poses.append(node_pose)
            print("path length is ", len(plan_))
        else:
            for node in plan_:
                node_pose = PoseStamped()
                node_pose.pose.position.x = node[0]
                node_pose.pose.position.y = node[1]
                plan.poses.append(node_pose)
            print("path length is ", len(plan_))
        return GetPlanResponse(plan)

    def costmap_sub_callback(self, data):
        #test
        print("Costmap recieved. Processing, please wait!")
        map_array_ = np.array(data.data,
                                   dtype=np.int8).reshape(data.info.height,
                                                          data.info.width)
        self.map_array = np.zeros((data.info.height, data.info.height))

        if (self.algo == 'informed_rrt' or self.algo=='rrt' or self.algo == 'rrt_star'):
            for i in range(1,len(map_array_[1])):
                for j in range(1, len(map_array_[0])):
                    point = map_array_[i, j]
                    if point > 0:
                        #if cost > 10 then assume it is obstacle
                        self.map_array[j, i] = 0
                    elif point < 0:
                        self.map_array[j, i] = 1
                    else:
                        self.map_array[j, i] = 1
        else:
            for i in range(1,len(map_array_[1])):
                for j in range(1, len(map_array_[0])):
                    point = map_array_[i, j]
                    if point > 0:
                        #if cost > 10 then assume it is obstacle
                        self.map_array[j, i] = 1
                    elif point < 0:
                        self.map_array[j, i] = 0
                    else:
                        self.map_array[j, i] = 0
        print("done processing, ready to plan ", self.algo)
        

    def __init__(self, algo, plot_on):
        rospy.init_node('rrt_planner')
        self.algo = algo
        self.plot_on = plot_on
        rospy.Subscriber("/map", OccupancyGrid, self.costmap_sub_callback)
        s = rospy.Service('rrt_get_plan', GetPlan, self.rrt_srv_callback)
        print("Ready to plan")
        self.map_array = []
        rospy.spin()

if __name__ == "__main__":
    # Load the map
    if (len(sys.argv) !=2 and len(sys.argv) !=3):
        print(" ",sys.argv, " ", len(sys.argv))
        print("Invalid usage! example usage: python main.py <algorithm> <plot_on:= 1/0>")
        print("choices of algorithms: astar, informed_rrt, rrt, rrt_star, dijkstra, bfs")
    else:
        if (len(sys.argv) == 2):
            planner = Global_Planner(sys.argv[1], 0)
        else:
            planner =  Global_Planner(sys.argv[1], sys.argv[2])
