#include <ros/ros.h>
#include "heima_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "heima_base_node");
    HeimaBase heima;
    ros::spin();
    return 0;
}
