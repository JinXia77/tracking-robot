#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('yolov3_transfer')
import sys
import rospy
import cv2
import time
import numpy as np
from pylab import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tracking_object import Detected_object
sys.path.append("..")
from my_yolo_track.msg import TrackInfos
from my_yolo_track.msg import TrackInfo


class image_converter:
 
    def __init__(self):
        self.net = cv2.dnn.readNet("../weights/yolov3.weights","../cfg/yolov3.cfg")
        self.classes = []
        with open("coco.names","r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        #print(classes)
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0,255,size = (len(self.classes),3))
        
        self.det_objs = TrackInfos()
        self.yolo_finish_flag = False
        self.color_dif_confidance = 0.1

        self.image_pub = rospy.Publisher("pub_image_yolov3",Image,queue_size=10)
        self.info_pub = rospy.Publisher("pub_track_info",TrackInfos,queue_size=10)
 
        self.bridge = CvBridge()
        # subscribe image
        self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback_rgb)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback_depth)
        self.font = cv2.FONT_HERSHEY_PLAIN
        self.starting_time = time.time()
        self.frame_id = 0           # frame counter, to calculate the fps by using (the number of frame) / (elapsed_time)
        self.track_counter = 0
        
  
    def callback_rgb(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.frame_id += 1
            #_, frame = cv_image
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow("image window",frame)
        (rows,cols,channels) = frame.shape
        
        #cv2.imshow("Image window", frame)
        # Detecting objects
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        print("rgb_image")
        print(cols)
        print(rows)
        
        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.2:
                    # Object detected
                    center_x = int(detection[0] * cols)
                    center_y = int(detection[1] * rows)
                    w = int(detection[2] * cols)
                    h = int(detection[3] * rows)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.8, 0.3)
        
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = self.colors[class_ids[i]]
                color_total_r = 0.0
                color_total_g = 0.0
                color_total_b = 0.0
                object_id = ""
                counter_pixel = 0

                a = x
                b = y   # index for getting all pixels color information in boxes
                print("x,y,w,h")
                print(x,y,w,h)

                # transversal all the pixels surrounded by this bounding box
                while(a <= x+w-1):
                    b = y
                    while(b <= y+h-1):
                        color_total = frame[b,a]
                        color_total_b += color_total[0]
                        color_total_g += color_total[1]
                        color_total_r += color_total[2]
                        b += 1
                        counter_pixel += 1
                    a +=1
             
                color_b_now = color_total_b / counter_pixel
                color_g_now = color_total_g / counter_pixel
                color_r_now = color_total_r / counter_pixel        # the average color(bgr) information in percentage of the detected boxes
                
                print("color_b_now,color_g_now,color_r_now")
                print(color_b_now,color_g_now,color_r_now)

                saved_object_flag = False
                det_objs_num = len(self.det_objs.tracking_objects)
                # start to compare two frames' color difference
                for j in range(0,det_objs_num,1):
                    tracking_object_self = self.det_objs.tracking_objects[j]
                    color_dif_b = abs((color_b_now - tracking_object_self.color_b)/tracking_object_self.color_b)
                    color_dif_g = abs((color_g_now - tracking_object_self.color_g)/tracking_object_self.color_g)
                    color_dif_r = abs((color_r_now - tracking_object_self.color_r)/tracking_object_self.color_r)
                    print("self_color")
                    print(tracking_object_self.color_b,tracking_object_self.color_g,tracking_object_self.color_r)
                    
                    if ((tracking_object_self.track_class == label) and (color_dif_b < self.color_dif_confidance) and 
                    (color_dif_g < self.color_dif_confidance) and (color_dif_r < self.color_dif_confidance)):
                        print("saved object detected!")
                        tracking_object_self.center_x = x + w/2
                        tracking_object_self.center_y = y + h/2
                        tracking_object_self.color_b = color_b_now
                        tracking_object_self.color_g = color_g_now
                        tracking_object_self.color_r = color_r_now
                        object_id = str(tracking_object_self.track_id)
                        saved_object_flag = True
                
                if(saved_object_flag == False):
                    print("no saved object detected, new object added")
                    self.track_counter += 1
                    det_obj = TrackInfo()
                    det_obj.center_x = x + w/2
                    det_obj.center_y = y + h/2
                    det_obj.track_class = label
                    det_obj.track_id = self.track_counter
                    det_obj.color_b = color_b_now
                    det_obj.color_g = color_g_now
                    det_obj.color_r = color_r_now
                    self.det_objs.tracking_objects.append(det_obj)
                    object_id = str(det_obj.track_id)

              
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, label + object_id + " " + str(round(confidence, 2)), (x, y + 30), self.font, 1, color, 1)
                
        
        elapsed_time = time.time() - self.starting_time
        fps = self.frame_id / elapsed_time
        cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), self.font, 4, (0, 0, 0), 3)

        cv2.imshow("Image window", frame)
        cv2.waitKey(3)
    
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)
        
        self.yolo_finish_flag = True


    def callback_depth(self,data):
        self.yolo_finish_flag = False
        #cv2.imshow("depth window",depth_image)
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "32FC1")
            
        except CvBridgeError as e:
            print(e)
        print("depth_image")
        print(data.width)
        print(data.height)
        image_width = data.width
        image_height = data.height

        while True:
            if(self.yolo_finish_flag == True):
                break
        print("while loop end!")
        print(self.det_objs.tracking_objects[0].track_class)
        for i in range(len(self.det_objs.tracking_objects)):
            tracking_depth_self = self.det_objs.tracking_objects[i]
            
            float_distance = frame[tracking_depth_self.center_y,tracking_depth_self.center_x]
            tracking_depth_self.distance = float_distance      
            print("distance")
            print(tracking_depth_self.distance)

        print("self.det_objs[]")
        print(self.det_objs)
        self.info_pub.publish(self.det_objs)

            

 
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)