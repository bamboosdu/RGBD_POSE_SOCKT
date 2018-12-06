#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <stdio.h> 
#include <stdlib.h> 
#include <cstdio>

double poseAMCLx, poseAMCLy ,poseAMCLz,poseAMCLX,poseAMCLY,poseAMCLZ,poseAMCLW;// 前三个参数表示pose，后面两个参数表示orientation
bool pose_ready=false;



void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
   //sbin printf("%s\n","to get pose!");
    poseAMCLx=msgAMCL->pose.pose.position.x;
    poseAMCLy=msgAMCL->pose.pose.position.y;
    poseAMCLZ=msgAMCL->pose.pose.orientation.z;
    poseAMCLW=msgAMCL->pose.pose.orientation.w;
    
    //这几个参数通常为零
    poseAMCLz=msgAMCL->pose.pose.position.z;
    
    poseAMCLX=msgAMCL->pose.pose.orientation.x;
    poseAMCLY=msgAMCL->pose.pose.orientation.y;
    // printf("%s\n","pose ready了!");
    pose_ready=true;
    //printf("我pose好了！\n");
    //ROS_INFO(msgAMCL);

}

bool getPose(int client_fd){
    
    float pose[7];
     //   while(1)
    //{
      
     try{
            
            //while(pose_ready==false){
           // ros::spinOnce();
           // }
            std::cout<<"Translation: ["<<poseAMCLx<<","<<poseAMCLy<<","<<poseAMCLz<<"]"<<std::endl;
            std::cout<<"Rotation:in Quaternion: ["<<poseAMCLX<<","<<poseAMCLY<<","<<poseAMCLZ<<","<<poseAMCLW<<"]"<<std::endl;
            pose[0] = poseAMCLx;
            pose[1] = poseAMCLy;
            pose[2] = poseAMCLz;
            pose[3] = poseAMCLX;
            pose[4] = poseAMCLY;
            pose[5] = poseAMCLZ;
            pose[6] = poseAMCLW;
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
            //continue;
           }
     // break;
     //   }

        pose_ready=false;

      
}

bool goalPose(int client_fd,float pose[7]){
    int data_len=7*sizeof(float);
    char* poseData=(char *)malloc(data_len);
    
    int rcv_len=recvData(client_fd,poseData,data_len);
    
    if(rcv_len<0){
        return false;
    }

    //copy the data from socket pkg
    int ind=0;
    
    for(int i=0;i<7;++i){
        memcpy(&pose[i],&poseData[ind],sizeof(float));
        ind+sizeof(float);
    }
    return true;

    //free(poseData);
    
}