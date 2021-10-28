# the discription of the detected objects

string      track_class     # the class of tracked object, people, desk, chair etc
uint32      track_id        # the index of tracked object such like people 1, people 2. same detected class with different index
uint32      center_x        # the centre point x of target
uint32      center_y        # centre point y of targe.  (x,y) point represent the detected object's centre pixel corridination
float32     distance        # the distance between target and camera
float32     color_b
float32     color_g
float32     color_r         # the color information 