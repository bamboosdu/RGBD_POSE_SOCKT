// ros msgs && opencv
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h> 
#include <stdlib.h> 
#include <cstdio>
// socket
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <pthread.h>
#include <netinet/tcp.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
//#include <octomap/octomap.h>

#include <fstream>
#include <sstream>


#include "socket.h"
#include "imageCallback.h"
#include "poseCallback.h"
using namespace std;

unsigned short portRGB= 3333; /*服务器监听端口号 发送rgbd*/
unsigned short portPOSE= 3332; /*服务器监听端口号 发送pose*/
unsigned short portGoal=3331;/*服务器监听端口号 接收pose*/
int sockfd,sockpd,sockgoal,server_fd,server_pd,sin_size; /*sock_fd:监听 socket;client_fd:数据传输 socket */
#define PI 3.1415926
// data offline rcv writer
ofstream ofs_off;
//movebase
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv){

    // run move_controller
    ros::init(argc, argv, "octo_navi");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//100
    
    image_transport::ImageTransport it(n);
    //订阅rgbd的话题
    image_transport::Subscriber subdepth = it.subscribe("/camera/depth/image_raw", 1, depthImageCallback);//camera/depth/image_raw
    image_transport::Subscriber subrgb = it.subscribe("/camera/rgb/image_raw", 1, rgbImageCallback);//camera/rgb/image_raw
   
    //订阅机器人当前pose的话题
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1, poseAMCLCallback);
   
    //发布pose到目标点话题

    MoveBaseClient ac("move_base", true);// tell the client we will spin a thread by default
    move_base_msgs::MoveBaseGoal goal;
    sockfd=initializeDataEngine(portRGB);//获得rgbd图传的套接字
    sockpd=initializeDataEngine(portPOSE);//获得pose的套接字
    sockgoal=initializeDataEngine(portGoal);//下个目标点

    while (1)
    {
        
        sin_size = sizeof(my_addr);

        printf("%s\n", "waiting for a connection");
     
///////////////////rgbd///////////////////////

        if ((server_fd = accept(sockfd, (struct sockaddr*)&remote_addr, (socklen_t *) &sin_size)) == -1)//接收客户端的连接
        {
            perror("accept");
            continue;
        }

        printf("%s\n", "received a rgbd connection");

///////////////////pose///////////////////////

        if ((server_pd = accept(sockpd, (struct sockaddr*)&remote_addr, (socklen_t *) &sin_size)) == -1)//接收客户端的连接
        {
            perror("accept");
            continue;
        }

        printf("%s\n", "received a pose connection");

///////////////////// disable nagle///////////////

        int flag = 1;
        if(setsockopt(server_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int))<0||setsockopt(server_pd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int))<0){
            printf("disable nagle failed\n");
        }
//////////////////判断是否获取到rgbd和pose////////////////////////////
         while(rgb_ready==false||depth_ready==false||pose_ready==false)
    {
       if(pose_ready==false)
            printf("%s\n","pose not ready!");
      // getchar();
       ros::spinOnce();
    }

        printf("ask for RGBD\n");
        getRGBD(server_fd);
        
        
        printf("ask for POSE\n"); 
        getPose(server_pd);
 ////////////////////////////获取要去的目标点////////////////////////////
    float *pose=new float[7];
    printf("ask for goal\n");
    bool goal_ready=goalPose(sockgoal,pose);
    
    if(goal_ready)  
    {
    printf("hi\n");
    goal.target_pose.header.frame_id = "/map";//send the goal point based on /map
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pose[0];
    goal.target_pose.pose.position.y = pose[1];
    goal.target_pose.pose.position.z = 0;
     printf("pose[%d]\n",pose[0]);
     printf("pose[%d]\n",pose[0]);
    goal.target_pose.pose.orientation.w = pose[6];
 
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
 
    ac.waitForResult();
 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Succeed!");
    else
        ROS_INFO("The base failed to move !");      

    }  
    printf("no more goal!\n");

    }
    return 0;
}