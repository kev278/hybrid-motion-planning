<<<<<<< HEAD
<<<<<<< HEAD
# Main and helper function

from mmap import MAP_SHARED
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
from astar import astar
class Global_Planner:
        
    def rrt_srv_callback(self, req):
        print("Starting RRT Planning. Start: ", req.start.pose.position.x, " ", req.start.pose.position.y, " Goal: ", req.goal.pose.position.x, " ", req.goal.pose.position.y)
        start = (int(req.start.pose.position.x), int(req.start.pose.position.y))
        goal  = (int(req.goal.pose.position.x), int(req.goal.pose.position.y))
        RRT_planner = RRT(self.map_array, start, goal)
        plan_ = RRT_planner.informed_RRT_star(n_pts=10000)
        #plan_ = astar(self.map_array, start, goal)
        plan = Path()
        for node in plan_:
            node_pose = PoseStamped()
            node_pose.pose.position.x = node[0]
            node_pose.pose.position.y = node[1]
            print("Node is ", node[1], " ", node[1])
            plan.poses.append(node_pose)
        print("path length is ", len(plan_))
        return GetPlanResponse(plan)

    def costmap_sub_callback(self, data):
        #test
        print("Costmap recieved. Processing, please wait!")
        self.map_array = np.array(data.data,
                                   dtype=np.int8).reshape(data.info.height,
                                                          data.info.width)
        for i in range(1,len(self.map_array[0])):
            for j in range(1, len(self.map_array[1])):
                point = self.map_array[i, j]
                if point > 2:
                    #if cost > 10 then assume it is obstacle
                    self.map_array[i, j] = 0
                elif point < 0:
                    self.map_array[i, j] = 0
                else:
                    self.map_array[i, j] = 1

        print("done processing, ready to plan")
        

    def __init__(self):
        rospy.init_node('rrt_planner')
        rospy.Subscriber("/map", OccupancyGrid, self.costmap_sub_callback)
        s = rospy.Service('rrt_get_plan', GetPlan, self.rrt_srv_callback)
        print("Ready to plan RRT")
        self.map_array = []
        rospy.spin()

if __name__ == "__main__":
    # Load the map
    planner = Global_Planner()
=======
=======
>>>>>>> 4b55fe67972983161b60dc4ad881b7d8afefdbb2
# Main and helper function

from PIL import Image
import numpy as np
from RRT import RRT

import matplotlib.pyplot as plt
from nav_msgs.srv import GetPlan
import rospy


        
def handle_add_two_ints(self,req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server(self):
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()
if __name__ == "__main__":
    # Load the map
    start = (200, 75)
    goal  = (30, 250)
    # map_array = load_map("WPI_map.jpg", 0.3)

    # Planning class
    RRT_planner = RRT(map_array, start, goal)

    # Search with RRT and RRT*
    # RRT_planner.RRT(n_pts=1000)
    # RRT_planner.RRT_star(n_pts=2000)
    RRT_planner.informed_RRT_star(n_pts=2000)
<<<<<<< HEAD
>>>>>>> 4b55fe67972983161b60dc4ad881b7d8afefdbb2
=======
>>>>>>> 4b55fe67972983161b60dc4ad881b7d8afefdbb2
