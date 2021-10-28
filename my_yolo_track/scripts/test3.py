import rospy
import time
import math
from std_msgs.msg import String
from mdl_people_tracker.msg import TrackedPersons2d
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from my_yolo_track.msg import TrackInfos
from my_yolo_track.msg import TrackInfo


input_name = ""
track_id = 0
distance = 0.0

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global input_name
    global track_id
    global distance
    
    rospy.init_node('start_tracking_node', anonymous=True)
    
    input_name = raw_input("Please input a object class as the tracking target >>")
    track_id = input("Please input the object index >>")
    
    
    distance = input("Please input the distance between people and turtlebot >>")

    twist = Twist()
    start_move = start_track()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()