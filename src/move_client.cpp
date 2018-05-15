#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
//#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setModelState_client");

  ros::NodeHandle n;
  //ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState");
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  
  geometry_msgs::Pose start_pose;
        start_pose.position.x = 0.0;
        start_pose.position.y = 0.0;
        start_pose.position.z = 2;
        start_pose.orientation.x = 0.0;
        start_pose.orientation.y = 0.0;
        start_pose.orientation.z = 0.0;
        start_pose.orientation.w = 0.0;

        geometry_msgs::Twist start_twist;
        start_twist.linear.x = 0.0;
        start_twist.linear.y = 0.0;
        start_twist.linear.z = 0.0;
        start_twist.angular.x = 0.0;
        start_twist.angular.y = 0.0;
        start_twist.angular.z = 0.0;

        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) "test";
        modelstate.reference_frame = (std::string) "world";
        modelstate.pose = start_pose;
        modelstate.twist = start_twist;
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  ROS_INFO("call server to move a model");
  if (client.call(setmodelstate))
  {
    ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
