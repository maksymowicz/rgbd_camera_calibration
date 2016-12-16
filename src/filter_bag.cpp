#include <ros/ros.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>

#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

using std::string;
using std::vector;
using std::copy;

int main(int argc, char** argv)
{
    string file_name;
    if (argc < 2) 
    {
        ROS_ERROR("Please supply bagfile name to parse.");
        return -1;
    }
    else 
        file_name = argv[1];

    rosbag::Bag bag; 
    bag.open(file_name.c_str(), rosbag::bagmode::Read);
    rosbag::View view(bag);

    sensor_msgs::Image::ConstPtr color, depth; 
    for (rosbag::MessageInstance m : view)
    {
        if (m.getTopic() == "/xtion/rgb/image_rect_color")
            color = m.instantiate<sensor_msgs::Image>();
        if (m.getTopic() == "/xtion/depth_registered/image_raw")
            depth = m.instantiate<sensor_msgs::Image>();
    }

    rosbag::Bag output;
    string out_name = file_name.substr(0,file_name.size()-4) + string("_filt.bag");

    output.open(out_name.c_str(), rosbag::bagmode::Write);
    output.write("/xtion/rgb/image_rect_color", color->header.stamp, color);
    output.write("/xtion/depth_registered/image_raw", depth->header.stamp, depth);

    output.close();
    bag.close();

    return 0;
}
