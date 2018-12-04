// ros msgs && opencv
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
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
// socket
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <pthread.h>
#include <netinet/tcp.h>

//#include <octomap/octomap.h>

#include <fstream>
#include <sstream>

using namespace std;

#define SERVPORT 3333 /*服务器监听端口号 */
int sockfd,client_fd,sin_size; /*sock_fd:监听 socket;client_fd:数据传输 socket */
struct sockaddr_in my_addr; /* 本机地址信息 */
struct sockaddr_in remote_addr; /* 客户端地址信息 */
const int MAXRECV = 10240;
#define BACKLOG 10 /* 最大同时连接请求数 */

#define PI 3.1415926

const int rbtnum = 1;
float camera_height = 1.1;

cv::Mat rgbImg;
cv::Mat shortImg(480, 640, CV_16UC1);
bool rgb_ready = false;
bool depth_ready = false;

vector<cv::Mat> crt_rgb_images(rbtnum);
vector<cv::Mat> crt_depth_images(rbtnum);
cv::Mat rgbImg0;
cv::Mat shortImg0(480, 640, CV_16UC1);


ros::ServiceClient gclient;
ros::ServiceClient sclient;



// data offline rcv writer
ofstream ofs_off;

void int2str(int n, char* ch)
{
    stringstream ss;
    string s;
    ss<<n;
    ss>>s;
    strcpy(ch, const_cast<char*>(s.c_str()));
} 
// 0 rgb
void rgbImageCallback0(const sensor_msgs::ImageConstPtr& msg){
    printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    // get rgbinfo into mat rgbImgImg
    try
    {
        printf("trans\n");
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg0 = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    crt_rgb_images[0] = rgbImg0;
    rgb_ready = true;
    //  show img
    //printf("show the rgb image.\n");
    cv::imshow("rgb", rgbImg0);
    //getchar();
    cv::waitKey(1000);
    //
}
// 0 depth
void depthImageCallback0(const sensor_msgs::ImageConstPtr& msg){
    //printf("depthImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    cv::Mat depthImg;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depthImg = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    // short img
    //printf("size of short: %lu\n", sizeof(short));
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            //uint8_t
            // origin
            shortImg0.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000);
        }
    }
    crt_depth_images[0] = shortImg0;
    depth_ready = true;
    //  show img
    //printf("show the depth image.\n");
    cv::imshow("short", shortImg0);
    //getchar();
    cv::waitKey(1000);
    //*/
}

//接收数据
int recvData(const int client_fd, char buf[], int len)
{
    memset (buf, 0, len);

    int i=0;
    while(i<len){
        char buf_tem[MAXRECV];
        memset (buf_tem, 0, MAXRECV);
        int status = recv(client_fd, buf_tem, MAXRECV, 0);
        memcpy(buf+i, buf_tem, status);
        i = i+status;

        //printf("i:%d\n", i);
        //printf("len:%d\n", len);
        //printf("status:%d\n", status);

        if ( status == -1 )
        {
            printf("status == -1 errno == %s in Socket::recv\n",errno);
            return 0;
        }
        else if( status == 0 )
        {
            //stop(client_fd);
            //return 0;
            break;
        }
        else if(len<=MAXRECV+1)
        {
            break;
        }
        /*else
        {
        return status;
        }*/
    }
}
//向指定客户端发送数据
bool sendData(const int client_fd, const char *ch, const int len)
{
    int status = send(client_fd, ch, len, 0);
    if ( status == -1 )
    {
        return false;
    }
    else
    {
        return true;
    }
}

// socket get rgbd
bool getRGBD(int client_fd){
    //ros::spinOnce();
    // rgb
    while(rgb_ready==false||depth_ready==false)
    {
       ros::spinOnce();
    }
    {
        int data_len = 480 * 640 * 3 * sizeof(uchar) * rbtnum;
        char* rgbData = (char *)malloc(data_len);
        printf("copy the rgb image\n");
        // for multi robot
        int ind = 0;
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            for (int i = 0; i < 480; ++i)
            {
                for (int j = 0; j < 640; ++j)
                {
                    //printf("%d,%d,%d\n",rid,i,j);
                   // printf("%s\n",&crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][0]);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
                    ind+=sizeof(uchar);
                    //printf("%d,%d,%d\n",ind,i,j);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
                    ind+=sizeof(uchar);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
                    ind+=sizeof(uchar);
                }
            }
        }


        sendData(client_fd, rgbData, data_len);
        printf("rgb send back\n");
        free(rgbData);
    }
    rgb_ready = false;
          
    // depth
    {
        int data_len = 480 * 640 * sizeof(short) * rbtnum;
        char* depthData = (char *)malloc(data_len);
        
        // for multi robot
        int ind = 0;
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            for (int i = 0; i < 480; ++i)
            {
                for (int j = 0; j < 640; ++j)
                {
                    memcpy(&depthData[ind], &crt_depth_images[rid].ptr<short>(i)[j], sizeof(short));
                    ind+=sizeof(short);
                }
            }
        }

        
        sendData(client_fd, depthData, data_len);
        printf("depth send back\n");
        free(depthData);
    }

    return true;
}


int main(int argc, char **argv){

    // run move_controller
    ros::init(argc, argv, "octo_navi");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    image_transport::ImageTransport it(n);
    // calibration
    //ros::Subscriber calib_sub = n.subscribe("camera/depth/camera_info", 1, infoCallback);
    // write
    ofs_off.open("/home/dsy/catkin_ws/src/virtual_scan/data/offline/pose_history.txt");
    //ofs_off.close();
    // topic: image
    image_transport::Subscriber subdepth = it.subscribe("camera/depth/image_raw", 1, depthImageCallback0);
    image_transport::Subscriber subrgb = it.subscribe("camera/rgb/image_raw", 1, rgbImageCallback0);
  
  

    // socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)//建立 socket
    {
        perror("socket");
        exit(1);
    }
    my_addr.sin_family=AF_INET;
    my_addr.sin_port=htons(SERVPORT);
    my_addr.sin_addr.s_addr = INADDR_ANY; //表示监听任何地址
    bzero(&(my_addr.sin_zero),8);
    //printf("ok\n");
    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) //将本机地址与建立的套接字号进行绑定
    {
        perror("bind");
        exit(1);
    }
    //printf("ok\n");
    if (listen(sockfd, BACKLOG) == -1) //开始监听
    {
        perror("listen");
        exit(1);
    }
    //printf("ok\n");

    // disable nagle
    int flag = 1;
    if(setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int))<0){
        printf("disable nagle failed\n");
    }

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

        printf("ask for rgbd\n");
        getRGBD(client_fd);

        usleep(100000);
    }
    return 0;
}