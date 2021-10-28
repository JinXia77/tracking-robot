#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from mdl_people_tracker.msg import TrackedPersons2d
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist

speed = 0.0     # global speed of turtlebot
turn = 0.0      # turning rate
name = ""
distance = -1
track_index = 0
saved_depth = 0.0       # if the people dismiss due to the obstcale from camera, save the lasted distance between camera and object, then move robot to the dismissing position
data_boxes = []

def get_time(depth):
    robot_speed = 0.1
    time = depth/robot_speed
    return time

def callback(data):
    #print data.boxes
    global speed
    global turn
    global name
    global distance
    global track_index
    global data_boxes

    data_length = len(data.boxes)
    data_boxes = data.boxes

    if(data_length):            # there is boxes transmitted here
        if(name == "" and distance == -1):

            id_flag = False             # id flag
            while(not id_flag):
                i = 0                   # index
                name = input("Please input a people id as the tracking target>>")
                distance = input("Please input the distance between people and turtlebot>>")
                
                for obj in data.boxes:
                    if(data.boxes[i].track_id == name):
                        print("track_id is found, target selected...")
                        id_flag = True
                        track_index = i
                        
                    else:
                        print("target searching...")
                        i+=1
        else:

            bbox_center = data.boxes[track_index].x + data.boxes[track_index].w/2
            if(data.boxes[track_index].depth > distance):
                speed = 0.1
                print("forward")
            if(data.boxes[track_index].depth < distance):
                speed = -0.1
                print("drawback")
            if(data.boxes[track_index].depth == distance):
                speed = 0.0
                print("stop")
            if(bbox_center > 320):
                print("TURN RIGHT")
                turn = -0.1
            if (bbox_center < 280):
                print("TURN LEFT")
                turn = 0.1
            if (bbox_center >=280 and bbox_center <= 320):
                print("CENTERED")
                turn = 0.0
    else:
        speed = 0.0
        turn = 0.0
        print("No data received...")

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('start_tracking_node', anonymous=True)
    global speed
    global turn
    global name
    global distance
    
    name = input("Please input a people id as the tracking target>>")
    distance = input("Please input the distance between people and turtlebot>>")

    twist = Twist()

    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)

    #rospy.Subscriber('/mdl_people_tracker/tracked_persons_2d ', TrackedPersons2d, callback)
    rospy.Subscriber("/mdl_people_tracker/tracked_persons_2d", TrackedPersons2d, callback)

    while not rospy.is_shutdown():

        # turn if we hit the line
        if ( turn != 0.0 or speed != 0.0):
            print("speed is %s" %speed)
            print("turn is %s" %turn)
            twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            turn = 0.0

            # straight otherwise
        else:
            print("stop %s" %speed)
            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            # send the message and delay
        pub.publish(twist)
        rospy.sleep(0.1)

    
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()