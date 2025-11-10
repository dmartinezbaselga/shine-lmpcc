#include <ros/ros.h>
#include <lmpcc_follower/follower.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    // Initialize a follower class and splin
    Follower follower_;
    if (!follower_.Initialize())
    {
        ROS_ERROR_STREAM_NAMED("Failed to initialize", ros::this_node::getName().c_str());
        exit(1);
    }
    else
    {
        ROS_INFO("Follower initialized");
        ros::spin(); // Regular spin
    }

    return 0;
}
