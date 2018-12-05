// ros msgs && opencv
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

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

unsigned short portRGB= 3333; /*服务器监听端口号 */
unsigned short portPOSE= 3332; /*服务器监听端口号 */
int sockfd,sockpd,server_fd,server_pd,sin_size; /*sock_fd:监听 socket;client_fd:数据传输 socket */
#define PI 3.1415926
// data offline rcv writer
ofstream ofs_off;


int main(int argc, char **argv){

    // run move_controller
    ros::init(argc, argv, "octo_navi");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//100
    
    image_transport::ImageTransport it(n);
    
    image_transport::Subscriber subdepth = it.subscribe("/camera/depth/image_raw", 1, depthImageCallback);//camera/depth/image_raw
    image_transport::Subscriber subrgb = it.subscribe("/camera/rgb/image_raw", 1, rgbImageCallback);//camera/rgb/image_raw
   
    
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1, poseAMCLCallback);
   
     sockfd=initializeDataEngine(portRGB);//获得rgbd图传的套接字
     sockpd=initializeDataEngine(portPOSE);//获得pose的套接字


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
//////////////////判断是否从rgbd和pose的topic中获得了需要的信息/////////////////////////
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
       

          

    }
    return 0;
}