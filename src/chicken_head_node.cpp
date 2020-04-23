#include "ros/ros.h"
#include "chicken_head.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "chicken_head_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    ChickenHead champ(nh, nh_private);
    
    ros::spin();
    return 0;
}