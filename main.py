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
