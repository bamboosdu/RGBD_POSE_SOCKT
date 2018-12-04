#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


bool getPose(int client_fd){
    tf::TransformListener listener;//listen for pose
    tf::StampedTransform transform;
    float pose[7];
        while(1)
    {
      
     try{
            listener.lookupTransform("/start_of_service", "/device",
                               ros::Time(0), transform);
            double yaw,pitch,roll;
            transform.getBasis().getRPY(roll,pitch,yaw);
            tf::Quaternion q=transform.getRotation();
            tf::Vector3 v=transform.getOrigin();
            std::cout<<"Translation: ["<<v.getX()<<","<<v.getY()<<","<<v.getZ()<<"]"<<std::endl;
            std::cout<<"Rotation:in Quaternion: ["<<q.getX()<<","<<q.getY()<<","<<q.getZ()<<","<<q.getW()<<"]"<<std::endl;
            pose[0] = v.getX();
            pose[1] = v.getY();
            pose[2] = v.getZ();
            pose[3] = q.getX();
            pose[4] = q.getY();
            pose[5] = q.getZ();
            pose[6] = q.getW();
            int data_len = 7 * sizeof(float);
            char* poseData = (char *)malloc(data_len);
            int ind = 0;
            for (int i = 0; i < 7; ++i)
            {
            memcpy(&poseData[ind], &pose[i], sizeof(float));
            ind+=sizeof(float);
            }
            sendData(client_fd, poseData, data_len);
            return true;
           }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
           }
      break;
        }
      
}