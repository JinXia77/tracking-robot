#!/usr/bin/env python
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

name = ""
track_id = 0
distance = 0.0

class start_track:

    def __init__(self):
        global name
        global distance
        global track_id
        self.pub = rospy.Publisher('~track_cmd_vel', Twist, queue_size=1)
        self.sub_track = rospy.Subscriber("/pub_track_info", TrackInfos, self.callback_track)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback_rgb)
        self.image_width = 0
        self.image_height = 0
        self.name_id_flag = False
        self.name_self = name
        self.distance_self = distance
        self.trackid_self = track_id
        self.track_index = 0
        self.twist_obj = Twist()
        self.det_objs = TrackInfos()
        self.saved_det_obj = TrackInfo()
        self.max_speed = 0.2
        self.max_turn = 0.2
        self.real_speed = 2.12
        self.real_turn = 2.01  # the speed and angular velocity for simulation world by testing

    def callback_track(self,data):
        self.det_objs = data
        object_num = len(self.det_objs.tracking_objects)
        self.name_id_flag = False
        while(self.name_id_flag==False):
            for i in range(0,object_num,1):
                if((self.name_self == self.det_objs.tracking_objects[i].track_class) and 
                (self.trackid_self == self.det_objs.tracking_objects[i].track_id)):
                    print("object detected!")
                    x = self.det_objs.tracking_objects[i].center_x
                    # y = det_objs.tracking_objects[i].center_y
                    self.track_index = i
                    self.saved_det_obj.track_class = self.name_self
                    self.saved_det_obj.track_id = self.trackid_self
                    self.saved_det_obj.center_x = self.det_objs.tracking_objects[i].center_x
                    self.saved_det_obj.distance = self.det_objs.tracking_objects[i].distance
                    self.move_to_obj(self.distance_self,x)
                    self.name_id_flag = True

            if(self.name_id_flag == False):
                # yes_or_no = input("Have you input correct name and id (y/n) >>")
                # if(yes_or_no == 'n'):
                if(self.saved_det_obj.track_class == "" and self.saved_det_obj.track_id == 0):
                    self.name_self = raw_input("No input name found, please input again >>")
                    self.trackid_self = input("No trackid found, please input again >>")
                else:
                    self.move_to_saved_obj(self.saved_det_obj.distance,self.saved_det_obj.center_x) # move to the latest location where the target disappear
                    find_obj_flag = False
                    turn_counter = 0
                    while(find_obj_flag == False):
                        self.turn_around(math.pi/6)         # each time turtlebot turn 30 degree ,also pi/6 in radians then the turn counter add 1
                        turn_counter += 1
                        if(turn_counter < 12):
                            for i in range(0,object_num,1):
                                if((self.name_self == self.det_objs.tracking_objects[i].track_class) and 
                                (self.trackid_self == self.det_objs.tracking_objects[i].track_id)):
                                    find_obj_flag = True
                                    print("saved target detected")
                                    self.twist_obj.linear.x = 0
                                    self.twist_obj.angular.z = 0
                                    self.pub.publish(self.twist_obj)
                                    time.sleep(1)       # sleep 1 second until turtlebot stop
                        else:
                            print("no saved object detected, mission failed")
                            self.twist_obj.linear.x = 0
                            self.twist_obj.angular.z = 0
                            self.pub.publish(self.twist_obj)
                            time.sleep(1)       # sleep 1 second until turtlebot stop
                            break

                                        

    def callback_rgb(self, data):
        self.image_width = data.width
        self.image_height = data.height
    
    def move_to_obj(self,distance,obj_x):
        if(distance < self.det_objs.tracking_objects[self.track_index]):
            print("FORWARD")
            self.twist_obj.linear.x = self.max_speed
        if(distance > self.det_objs.tracking_objects[self.track_index]):
            print("DRAWBACK")
            self.twist_obj.linear.x = -self.max_speed
        if(obj_x > self.image_width/2):
            print("Turn Right")
            self.twist_obj.angular.z = -self.max_turn
        if(obj_x < self.image_width/2):
            print("Turn Left")
            self.twist_obj.angular.z = self.max_turn
        
        self.pub.publish(self.twist_obj)
    
    def move_to_saved_obj(self,distance,x):
        angle = self.get_angle(distance,x)
        self.turn_around(angle)     # first turn round to make the disappear position center on vision
        self.move_forward(distance)     # second move forward for distance and approach the disappear position

    def move_forward(self,distance):
        self.twist_obj.linear.x = self.max_speed
        self.twist_obj.angular.z = 0
        print("forward")
        self.pub.publish(self.twist_obj)
        time.sleep(distance/self.real_speed)

    def turn_around(self,angle):
        self.twist_obj.linear.x = 0
        self.twist_obj.linear.z = self.max_turn
        self.pub.publish(self.twist_obj)
        print("turn")
        time.sleep(angle/self.real_turn)

    def get_angle(self,distance,x):
        a = self.image_width - x
        angel = math.asin(a/distance)   # represent with radian 
        return angle    

    

    
def main():

    global name
    global track_id
    global distance
    
    rospy.init_node('start_tracking_node', anonymous=True)
    
    name = raw_input("Please input a object class as the tracking target >>")
    track_id = input("Please input the object index >>")
    distance = input("Please input the distance between people and turtlebot >>")

    twist = Twist()
    start_move = start_track()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()