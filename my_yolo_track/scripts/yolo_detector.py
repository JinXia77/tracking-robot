#!/usr/bin/env python
import cv2
import numpy as np


# load yolo
#net = cv2.dnn.readNet(net_weight_path,net_cfg_path)
net = cv2.dnn.readNet("weights/yolov3.weights","cfg/yolov3.cfg")
classes = []
with open("coco.names","r") as f:
    classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0,255,size=(len(classes),3))

#load image
img = cv2.imread("room_ser.jpg")
img = cv2.resize(img,None,fx=0.4,fy=0.4)
height,width,channels = img.shape
#print(height,width,channels)

# Detecting objects
#blob = cv2.dnn.blobFromImage(img,0.00392,(416,416),(0,0,0),True,crop=False)
blob = cv2.dnn.blobFromImage(img,1,(416,416),(0,0,0),True,crop=False)
net.setInput(blob)
outs = net.forward(output_layers)
print("outs:")
print(outs)

# showing information on the screen
class_ids = []
confidances = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidance = scores[class_id]
        if confidance > 0.5:
            # object detected
            center_x = int(detection[0]*width)
            center_y = int(detection[1]*height)
            w = int(detection[2]*width)
            h = int(detection[3]*height)

            # Rectangle coordinates
            x = int(center_x - w/2)
            y = int(center_y - h/2)

            boxes.append([x,y,w,h])
            confidances.append(float(confidance))
            class_ids.append(class_id)
            print("detection:")
            print(detection)
# reduce noise frames
#indexes = cv2.dnn.NMSBoxes(boxes,confidances,0.5,0.4)

font = cv2.FONT_HERSHEY_PLAIN
# for i in range(len(boxes)):
#     if i in indexes:
#         x,y,w,h = boxes[i]
#         label = str(classes[class_ids[i]])
#         color = colors[i]
#         cv2.rectangle(img, (x,y), (x+w,y+h), color, 2)
#         cv2.putText(img, label, (x,y+30), font, 3, color, 3)

for i in range(len(boxes)):
    x,y,w,h = boxes[i]
    label = str(classes[class_ids[i]])
    color = colors[i]
    cv2.rectangle(img, (x,y), (x+w,y+h), color, 2)
    cv2.putText(img, label, (x,y+30), font, 3, color, 3)



cv2.imshow("Image",img)
cv2.waitKey(0)
cv2.destroyAllWindows()