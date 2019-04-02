#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <stdio.h> 
#include <stdlib.h> 
#include <cstdio>

//const int MAXRECV=10240;
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
////////////////接收来自主机的pose/////////////////////////
bool goalPose(int client_fd,float pose[1][7]){
    int data_len=7*sizeof(float);
    char* poseData=(char *)malloc(data_len);
    float pose_test[7];
    char pkgData[MAXRECV];
    int rcv_len=recv(client_fd,pkgData,data_len,0);
    
    if(rcv_len<0){
        printf("no pose from win pc!\n");
        return false;
    }
   
    memcpy(&poseData[0],&pkgData,rcv_len);

    //copy the data from socket pkg
    int ind=0;
    for(int i=0;i<7;i++){
        //printf("the posedata[]=%d\n",poseData[i]);
        memcpy(&pose[0][i],&poseData[ind],sizeof(float));
       // memcpy(&pose_test[i],&poseData[ind],sizeof(float));
        ind+=sizeof(float);
    }
    std::cout<<"Translation: ["<<pose[0][0]<<","<<pose[0][1]<<","<<pose[0][2]<<"]"<<std::endl;
    std::cout<<"Rotation:in Quaternion: ["<<pose[0][3]<<","<<pose[0][4]<<","<<pose[0][5]<<","<<pose[0][6]<<"]"<<std::endl;

    // std::cout<<"Translation: ["<<pose_test[0]<<","<<pose_test[1]<<","<<pose_test[2]<<"]"<<std::endl;
    // std::cout<<"Rotation:in Quaternion: ["<<pose_test[3]<<","<<pose_test[4]<<","<<pose_test[5]<<","<<pose_test[6]<<"]"<<std::endl;
    delete poseData;
    return true;

    //free(poseData);
    
}
bool sendSucceed(int client_fd){
    
    
      
     try{
            int data_len=sizeof(char);

            char state='t';
            
            char posedata[64];
            memcpy(posedata, &state, sizeof(char));
           
            sendData(client_fd, posedata, data_len);
            printf("send succeed to windows!");
            return true;
           }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            //continue;
           }
  
       }

bool sendFailure(int client_fd){
    
    
      
     try{
            int data_len=sizeof(char);

            char state='f';
            
            char posedata[64];
            memcpy(posedata, &state, sizeof(char));
           
            sendData(client_fd, posedata, data_len);
            printf("send failure to windows!");
            return true;
           }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            //continue;
           }
  
       }

      
