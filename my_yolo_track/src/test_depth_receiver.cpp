#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <my_yolo_track/TrackInfo.h>
#include <my_yolo_track/TrackInfos.h>



static const uint32_t MY_ROS_QUEUE_SIZE = 1000;
my_yolo_track::TrackInfos det_objs;
sensor_msgs::Image depth_info;
int det_num;
int depth_image_width;
bool track_finish_flag = false;
ros::Publisher trackinfo_pub;

void depthcb(const sensor_msgs::Image::ConstPtr& msg)
{
    // The message's data is a raw buffer. While the type is uint8_t*, the
    // data itself is an array of floats (for depth data), the value of
    // the float being the distance in meters.
    while(!track_finish_flag); //wait until the trackcb function finished

    ROS_INFO("depth info received");
    int x,y,index,received_num;
    float distance;
    received_num = det_objs.tracking_objects.size();
    depth_image_width = msg->width;

    for(int i = 0; i < received_num; i++)
    {
        x = det_objs.tracking_objects[i].center_x;
        y = det_objs.tracking_objects[i].center_y;
        index = x + y*depth_image_width;
        distance = *reinterpret_cast<const float*>(&msg->data[index]);
        det_objs.tracking_objects[i].distance = distance;
        int id = det_objs.tracking_objects[i].track_id;
        ROS_INFO("the distance of person %d is %f",&id,&distance);
    }
    //std::cout << "Top-left corner: " << *reinterpret_cast<const float*>(&msg->data[0]) << "m" << std::endl;
    //depth_info = msg;
    trackinfo_pub.publish(det_objs);

}
void trackcb(const my_yolo_track::TrackInfos::ConstPtr& msg)
{
    ROS_INFO("I heard pub_track_info!");
    //track_info = msg;
    //det_num = sizeof(msg->tracking_objects)/sizeof(msg->tracking_objects[0])
    det_num = msg->tracking_objects.size();      //return the number of elements in msg array
    for(int i = 0; i < det_num; i++)
    {
        my_yolo_track::TrackInfo det_obj;
        det_obj.track_class = msg->tracking_objects[i].track_class;
        det_obj.track_id = msg->tracking_objects[i].track_id;
        det_obj.center_x = msg->tracking_objects[i].center_x;
        det_obj.center_y = msg->tracking_objects[i].center_y;
        det_obj.distance = msg->tracking_objects[i].distance;
        det_obj.color_b = msg->tracking_objects[i].color_b;
        det_obj.color_g = msg->tracking_objects[i].color_g;
        det_obj.color_r = msg->tracking_objects[i].color_r;
        det_objs.tracking_objects.push_back(det_obj);
    }
    track_finish_flag = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    std::cout << "here is depth data processing" << std::endl;
    ros::NodeHandle nh;
    
    ros::Subscriber sub_depth = nh.subscribe("camera/depth/image_raw", MY_ROS_QUEUE_SIZE, depthcb);
    ros::Subscriber sub_track_info = nh.subscribe("pub_track_info",MY_ROS_QUEUE_SIZE,trackcb);

    trackinfo_pub = nh.advertise<my_yolo_track::TrackInfos>("track_information",MY_ROS_QUEUE_SIZE);
    ros::spin();
    return 0;
}
