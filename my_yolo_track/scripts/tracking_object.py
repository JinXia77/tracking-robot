#!/usr/bin/env python

class Detected_object:

    def __init__(self):
        self.track_class = ""   # the class of tracked object, people, desk, chair etc
        self.track_id = 0       # the index of tracked object such like people 1, people 2. same detected class with different index
        self.center_x = 0       # the centre point x of target
        self.center_y = 0       # centre point y of targe.  (x,y) point represent the detected object's centre pixel corridination
        self.distance = 0.0     # the distance between target and camera
        self.color_r = 1.0
        self.color_g = 1.0
        self.color_b = 1.0       # the color information of detected object with percentage