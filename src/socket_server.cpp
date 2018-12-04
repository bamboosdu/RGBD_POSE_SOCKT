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
using namespace std;

unsigned short port= 3333; /*服务器监听端口号 */
int sockfd,client_fd,sin_size; /*sock_fd:监听 socket;client_fd:数据传输 socket */
#define PI 3.1415926
// data offline rcv writer
ofstream ofs_off;


int main(int argc, char **argv){

    // run move_controller
    ros::init(argc, argv, "octo_navi");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//100
    
    image_transport::ImageTransport it(n);
    // calibration
    //ros::Subscriber calib_sub = n.subscribe("camera/depth/camera_info", 1, infoCallback);
    // write
    //ofs_off.open("/home/dsy/catkin_ws/src/virtual_scan/data/offline/pose_history.txt");
    //ofs_off.close();
    // topic: image

    image_transport::Subscriber subdepth = it.subscribe("/camera/depth/image_raw", 1, depthImageCallback);//camera/depth/image_raw
    image_transport::Subscriber subrgb = it.subscribe("/camera/rgb/image_raw", 1, rgbImageCallback);//camera/rgb/image_raw
  
     sockfd=initializeDataEngine(port);


    while (1)
    {
        
        sin_size = sizeof(my_addr);

        printf("%s\n", "waiting for a connection");

        if ((client_fd = accept(sockfd, (struct sockaddr*)&remote_addr, (socklen_t *) &sin_size)) == -1)//接收客户端的连接
        {
            perror("accept");
            continue;
        }

        printf("%s\n", "received a connection");
        
        // disable nagle
        int flag = 1;
        if(setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int))<0){
            printf("disable nagle failed\n");
        }

        printf("ask for RGBD\n");
        getRGBD(client_fd);
        /*printf("ask for POSE\n");
        if(n.ok())
        {
    
        getPose(client_fd);
       
<<<<<<< HEAD
        }*/       

    }
    return 0;
}