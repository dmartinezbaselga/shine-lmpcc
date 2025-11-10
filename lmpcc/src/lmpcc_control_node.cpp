#include <ros/ros.h>
#include <lmpcc/lmpcc_controller.h>

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, ros::this_node::getName());

    // Seperate node handle and callback queue for mpcc follower
    // ros::NodeHandle nh_follower;
    // ros::CallbackQueue callback_queue_follower;
    // nh_follower.setCallbackQueue(&callback_queue_follower);

    MPCC controller_;

    // initialize predictive control node
    if (!controller_.initialize())
    {
      ROS_ERROR_STREAM_NAMED("FILED TO INITIALIZE %s", ros::this_node::getName().c_str());
      exit(1);
    }
    else
    {
      // spin node, till ROS node is running on
      ROS_INFO_STREAM_NAMED("%s INITIALIZE SUCCESSFULLY!!", ros::this_node::getName().c_str());

      // std::thread spinner_thread_follower([&callback_queue_follower]()
      //                                     {
      //   ros::SingleThreadedSpinner spinner_follower;
      //   spinner_follower.spin(&callback_queue_follower); });

      ros::spin(); // Regular spin

      // spinner_thread_follower.join();
    }
  }

  catch (ros::Exception &e)
  {
    ROS_ERROR("LMPCC: Error occured: %s ", e.what());
    exit(1);
  }

  return 0;
}
