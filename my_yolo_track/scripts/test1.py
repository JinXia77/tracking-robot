#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('yolov3_transfer')
import sys
import rospy
import cv2
import time
import numpy as np
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
        
        self.det_obj = TrackInfo()
        self.det_objs = []
        self.yolo_finish_flag = False
        self.color_dif_confidance = 0.5

        self.image_pub = rospy.Publisher("pub_image_yolov3",Image,queue_size=10)
        self.info_pub = rospy.Publisher("pub_track_info",TrackInfos,queue_size=10)
 
        self.bridge = CvBridge()
        # subscribe image
        self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback_rgb)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback_depth)
        self.font = cv2.FONT_HERSHEY_PLAIN
        self.starting_time = time.time()
        self.frame_id = 0           # frame counter, to calculate the fps by using (the number of frame) / (elapsed_time)
        
  
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
        # print("indexes are")
        # print(indexes)
        # print("class_ids")
        # print(class_ids)
        # print("range len boxes")
        # print(range(len(boxes)))

        # print("self.classes are")
        # print(self.classes)
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                # print("classid[i] and classes[classid[i]]")
                # print(class_ids[i])
                # class_num = class_ids[i] + 1
                # print(self.classes[class_num])
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = self.colors[class_ids[i]]
                
                color_total_r = 0.0
                color_total_g = 0.0
                color_total_b = 0.0
                object_id = ""
                color_total = 0
                surface = w*h
                counter_pixel = 0

                a = x
                b = y   # index for getting all pixels color information in boxes
                print("x,y,w,h")
                print(x,y,w,h)

                while(a <= x+w-1):
                    b = y
                    while(b <= y+h-1):
                        # pixel_index = a + b*w
                        # color_total = frame[pixel_index]
                        color_total = frame[b,a]
                        color_total_b += color_total[0]
                        color_total_g += color_total[1]
                        color_total_r += color_total[2]
                        b += 1
                        counter_pixel += 1
                    a +=1
                # print("w and h")
                # print(w,h)
                # print("counter_pixel and surface")
                # print(counter_pixel,surface)
                
                # print("color total")
                # print(color_total_b,color_total_g,color_total_r)    
                color_b_now = color_total_b / counter_pixel
                color_g_now = color_total_g / counter_pixel
                color_r_now = color_total_r / counter_pixel        # the average color(bgr) information in percentage of the detected boxes
                
                print("color_b_now,color_g_now,color_r_now")
                print(color_b_now,color_g_now,color_r_now)

                saved_object_flag = False
                # start to compare two frames' color difference
                for j in range(len(self.det_objs)):
                    color_dif_b = abs((color_b_now - self.det_objs[j].color_b)/self.det_objs[j].color_b)
                    color_dif_g = abs((color_g_now - self.det_objs[j].color_g)/self.det_objs[j].color_g)
                    color_dif_r = abs((color_r_now - self.det_objs[j].color_r)/self.det_objs[j].color_r)
                    print("self_color")
                    print(self.det_objs[j].color_b,self.det_objs[j].color_g,self.det_objs[j].color_r)
                    # print("color_dif_b,color_dif_g,color_dif_r")
                    # print(color_dif_b,color_dif_g,color_dif_r)
                    
                    if ((self.det_objs[j].track_class == label) and (color_dif_b < self.color_dif_confidance) and (color_dif_g < self.color_dif_confidance) and (color_dif_r < self.color_dif_confidance)):
                        print("saved object detected!")
                        self.det_objs[j].center_x = x + w/2
                        self.det_objs[j].center_y = y + h/2
                        self.det_objs[j].color_b = color_b_now
                        self.det_objs[j].color_g = color_g_now
                        self.det_objs[j].color_r = color_r_now
                        object_id = str(self.det_objs[j].track_id)
                        saved_object_flag = True
                
                if(saved_object_flag == False):
                    print("no saved object detected, new object added")
                    self.det_obj.center_x = x + w/2
                    self.det_obj.center_y = y + h/2
                    self.det_obj.track_class = label
                    self.det_obj.track_id += 1
                    self.det_obj.color_b = color_b_now
                    self.det_obj.color_g = color_g_now
                    self.det_obj.color_r = color_r_now
                    self.det_objs.append(self.det_obj)
                    object_id = str(self.det_obj.track_id)

                #print(self.det_objs)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, label + object_id + " " + str(round(confidence, 2)), (x, y + 30), self.font, 2, color, 3)
                
        
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
        #print("convert success!")
        print("depth_image")
        print(data.width)
        print(data.height)
        image_width = data.width
        image_height = data.height
        # print(frame.data)
        #print(data.data)
        # wait until the rgb yolo detection finished
        while True:
            if(self.yolo_finish_flag == True):
                break
        print("while loop end!")
        print(self.det_objs[0].track_class)
        for i in range(len(self.det_objs)):
            detected_centre_index = self.det_objs[i].center_x + self.det_objs[i].center_y * image_width
            print("detected_centre_index")
            print(detected_centre_index)
            str_distance = data.data[detected_centre_index]
            #float_distance = float(uint8_distance)      # need reinterpret_cast to conver unit8 type to float type, need to conver all to C++
            uint8_distance = int(str_distance)
            float_distance = float(uint8_distance)
            self.det_objs[i].distance = float_distance
            #self.det_objs[i].distance = self.reinterpret_cast_like(uint8_distance,float)
            print("distance")
            print(self.det_objs[i].distance)

        print("self.det_objs[]")
        print(self.det_objs)
        self.info_pub.publish(self.det_objs)

    # def reinterpret_cast_like(self,instance,cls):
    #     newinstance=instance
    #     newinstance.__class__ = cls
    #     return newinstance
            

 
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