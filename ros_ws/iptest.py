import sys
import rospy

from baxter_interface import *

sys.path.insert(0, '/home/drl/ros_ws/src/baxter-soft-hand/scripts/softhandpy')
from softhandpy.TubeSoftHand import * # SoftHand, HandType

rospy.init_node("testgrip")


s = SoftHand()
s.close_value = 1.0 # good for any object grasping
s.Open()
