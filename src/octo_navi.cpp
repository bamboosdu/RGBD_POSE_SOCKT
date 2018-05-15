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

const int rbtnum = 3;
float camera_height = 1.1;

cv::Mat rgbImg;
cv::Mat shortImg(480, 640, CV_16UC1);
bool rgb_ready = false;
bool depth_ready = false;

vector<cv::Mat> crt_rgb_images(rbtnum);
vector<cv::Mat> crt_depth_images(rbtnum);
cv::Mat rgbImg0;
cv::Mat shortImg0(480, 640, CV_16UC1);
cv::Mat rgbImg1;
cv::Mat shortImg1(480, 640, CV_16UC1);
cv::Mat rgbImg2;
cv::Mat shortImg2(480, 640, CV_16UC1);
cv::Mat rgbImg3;
cv::Mat shortImg3(480, 640, CV_16UC1);
cv::Mat rgbImg4;
cv::Mat shortImg4(480, 640, CV_16UC1);

ros::ServiceClient gclient;
ros::ServiceClient sclient;
float pose[7];
bool pose_ready = false;

const float camera_factor = 1000;
const float camera_cx = 320.500000;
const float camera_cy = 240.500000;
const float camera_fx = 554.382713;
const float camera_fy = 554.382713;

const int times = 1.0;
float rbt_v = 1.0/times - 0.01;

// 每次移动走路径的比例
const double move_distance_rate = 1;

vector< vector< float > > objective_positions;

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
    //printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg0 = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    crt_rgb_images[0] = rgbImg0;
    rgb_ready = true;
    /*//  show img
    cv::imshow("rgb", rgbImg0);
    cv::waitKey(1);
    //*/
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
    /*//  show img
    cv::imshow("short", shortImg0);
    cv::waitKey(1);
    //*/
}
// 1 rgb
void rgbImageCallback1(const sensor_msgs::ImageConstPtr& msg){
    //printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg1 = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    crt_rgb_images[1] = rgbImg1;
    rgb_ready = true;
    /*//  show img
    cv::imshow("rgb", rgbImg1);
    cv::waitKey(1);
    //*/
}
// 1 depth
void depthImageCallback1(const sensor_msgs::ImageConstPtr& msg){
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
            shortImg1.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000);
        }
    }
    crt_depth_images[1] = shortImg1;
    depth_ready = true;
    /*//  show img
    cv::imshow("short", shortImg1);
    cv::waitKey(1);
    //*/
}
// 2 rgb
void rgbImageCallback2(const sensor_msgs::ImageConstPtr& msg){
    //printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg2 = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    crt_rgb_images[2] = rgbImg2;
    rgb_ready = true;
    /*//  show img
    cv::imshow("rgb", rgbImg2);
    cv::waitKey(1);
    //*/
}
// 2 depth
void depthImageCallback2(const sensor_msgs::ImageConstPtr& msg){
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
            shortImg2.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000);
        }
    }
    crt_depth_images[2] = shortImg2;
    depth_ready = true;
    /*//  show img
    cv::imshow("short", shortImg2);
    cv::waitKey(1);
    //*/
}

// 3 rgb
void rgbImageCallback3(const sensor_msgs::ImageConstPtr& msg){
    //printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg3 = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    crt_rgb_images[3] = rgbImg3;
    rgb_ready = true;
    /*//  show img
    cv::imshow("rgb", rgbImg2);
    cv::waitKey(1);
    //*/
}
// 3 depth
void depthImageCallback3(const sensor_msgs::ImageConstPtr& msg){
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
            shortImg3.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000);
        }
    }
    crt_depth_images[3] = shortImg3;
    depth_ready = true;
    /*//  show img
    cv::imshow("short", shortImg2);
    cv::waitKey(1);
    //*/
}

// 4 rgb
void rgbImageCallback4(const sensor_msgs::ImageConstPtr& msg){
    //printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg4 = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    crt_rgb_images[4] = rgbImg4;
    rgb_ready = true;
    /*//  show img
    cv::imshow("rgb", rgbImg2);
    cv::waitKey(1);
    //*/
}
// 4 depth
void depthImageCallback4(const sensor_msgs::ImageConstPtr& msg){
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
            shortImg4.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000);
        }
    }
    crt_depth_images[4] = shortImg4;
    depth_ready = true;
    /*//  show img
    cv::imshow("short", shortImg2);
    cv::waitKey(1);
    //*/
}

void callForPose(int id){
    gazebo_msgs::GetModelState getmodelstate;
    stringstream ss;
    string s;
    ss<<"test"<<id;
    ss>>s;
    getmodelstate.request.model_name = (std::string) s;
    //ROS_INFO("call server to get pose");
    if (gclient.call(getmodelstate))
    {
      //ROS_INFO("success");
    }
    else
    {
      ROS_ERROR("Failed to call service");
    }
    //printf("position x: %f\n", getmodelstate.response.pose.position.x);
    pose[0] = getmodelstate.response.pose.position.x;
    pose[1] = getmodelstate.response.pose.position.y;
    pose[2] = getmodelstate.response.pose.position.z;
    pose[3] = getmodelstate.response.pose.orientation.x;
    pose[4] = getmodelstate.response.pose.orientation.y;
    pose[5] = getmodelstate.response.pose.orientation.z;
    pose[6] = getmodelstate.response.pose.orientation.w;
    //printf("call and get pose of robot%d q:\n w: %f, x: %f, y: %f, z: %f\n", id, pose[6], pose[3], pose[4], pose[5]);
    pose_ready = true;
}

void setForPose(int id, float x, float y, float z, float qx, float qy, float qz, float qw){
    gazebo_msgs::SetModelState setmodelstate;
    stringstream ss;
    string s;
    ss<<"test"<<id;
    ss>>s;
    geometry_msgs::Pose posemsg;
        posemsg.position.x = x;
        posemsg.position.y = y;
        posemsg.position.z = z;
        posemsg.orientation.x = qx;
        posemsg.orientation.y = qy;
        posemsg.orientation.z = qz;
        posemsg.orientation.w = qw;
    gazebo_msgs::ModelState modelstate;
        modelstate.pose = posemsg;
        modelstate.model_name = (std::string) s;
    setmodelstate.request.model_state = modelstate;
    if (sclient.call(setmodelstate)){}
    else
        ROS_ERROR("Failed to call service");
}

void infoCallback(const sensor_msgs::CameraInfoConstPtr &msg){
    
    //printf("cx: %lf, cy: %lf, fx: %lf, fy: %lf\n", msg->K[2], msg->K[5], msg->K[0], msg->K[4]);

    //printf("fx: %lf\n", msg->K[0]);
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
    ros::spinOnce();
    // rgb
    {
        int data_len = 480 * 640 * 3 * sizeof(uchar) * rbtnum;
        char* rgbData = (char *)malloc(data_len);
        
        // for multi robot
        int ind = 0;
        for (int rid = 0; rid < rbtnum; ++rid)
        {
            for (int i = 0; i < 480; ++i)
            {
                for (int j = 0; j < 640; ++j)
                {
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
                    ind+=sizeof(uchar);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
                    ind+=sizeof(uchar);
                    memcpy(&rgbData[ind], &crt_rgb_images[rid].ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
                    ind+=sizeof(uchar);
                }
            }
        }

        // // rbt0
        // int ind = 0;
        // for (int i = 0; i < 480; ++i)
        // {
        //     for (int j = 0; j < 640; ++j)
        //     {
        //         memcpy(&rgbData[ind], &rgbImg0.ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //         memcpy(&rgbData[ind], &rgbImg0.ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //         memcpy(&rgbData[ind], &rgbImg0.ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //     }
        // }
        // // rbt1
        // for (int i = 0; i < 480; ++i)
        // {
        //     for (int j = 0; j < 640; ++j)
        //     {
        //         memcpy(&rgbData[ind], &rgbImg1.ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //         memcpy(&rgbData[ind], &rgbImg1.ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //         memcpy(&rgbData[ind], &rgbImg1.ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //     }
        // }
        // // rbt2
        // for (int i = 0; i < 480; ++i)
        // {
        //     for (int j = 0; j < 640; ++j)
        //     {
        //         memcpy(&rgbData[ind], &rgbImg2.ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //         memcpy(&rgbData[ind], &rgbImg2.ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //         memcpy(&rgbData[ind], &rgbImg2.ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
        //         ind+=sizeof(uchar);
        //     }
        // }

        sendData(client_fd, rgbData, data_len);
        printf("rgb send back\n");
        free(rgbData);
    }   
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

        // // rbt0
        // int ind = 0;
        // for (int i = 0; i < 480; ++i)
        // {
        //     for (int j = 0; j < 640; ++j)
        //     {
        //         memcpy(&depthData[ind], &shortImg0.ptr<short>(i)[j], sizeof(short));
        //         ind+=sizeof(short);
        //     }
        // }
        // // rbt1
        // for (int i = 0; i < 480; ++i)
        // {
        //     for (int j = 0; j < 640; ++j)
        //     {
        //         memcpy(&depthData[ind], &shortImg1.ptr<short>(i)[j], sizeof(short));
        //         ind+=sizeof(short);
        //     }
        // }
        // // rbt2
        // for (int i = 0; i < 480; ++i)
        // {
        //     for (int j = 0; j < 640; ++j)
        //     {
        //         memcpy(&depthData[ind], &shortImg2.ptr<short>(i)[j], sizeof(short));
        //         ind+=sizeof(short);
        //     }
        // }

        sendData(client_fd, depthData, data_len);
        printf("depth send back\n");
        free(depthData);
    }

    return true;
}

// socket get depth
bool getDepth(int client_fd){
    ros::spinOnce();
    //char* depthData = (char *)malloc(480 * 640 * (sizeof(short)) * rbtnum);
    int data_len = 480 * 640 * sizeof(short) * rbtnum;
    char* depthData = (char *)malloc(data_len);
    // rbt0
    int ind = 0;
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            memcpy(&depthData[ind], &shortImg0.ptr<short>(i)[j], sizeof(short));
            ind+=sizeof(short);
        }
    }
    // rbt1
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            memcpy(&depthData[ind], &shortImg1.ptr<short>(i)[j], sizeof(short));
            ind+=sizeof(short);
        }
    }
    // rbt2
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            memcpy(&depthData[ind], &shortImg2.ptr<short>(i)[j], sizeof(short));
            ind+=sizeof(short);
        }
    }
    sendData(client_fd, depthData, data_len);
    printf("depth send back\n");
    free(depthData);
    return true;
}

// socket get pose
bool getPose(int client_fd){
    int data_len = 7 * sizeof(float) * rbtnum;
    char* poseData = (char *)malloc(data_len);
   
    // for multi robot
    int ind = 0;
    for (int rid = 0; rid < rbtnum; ++rid)
    {
        callForPose(rid);
        for (int i = 0; i < 7; ++i)
        {
            memcpy(&poseData[ind], &pose[i], sizeof(float));
            ind+=sizeof(float);
        }
    }

    // int ind = 0;
    // // rbt0
    // callForPose(0);
    // for (int i = 0; i < 7; ++i)
    // {
    //     memcpy(&poseData[ind], &pose[i], sizeof(float));
    //     ind+=sizeof(float);
    // }
    // // rbt1
    // callForPose(1);
    // for (int i = 0; i < 7; ++i)
    // {
    //     memcpy(&poseData[ind], &pose[i], sizeof(float));
    //     ind+=sizeof(float);
    // }
    // // rbt2
    // callForPose(2);
    // for (int i = 0; i < 7; ++i)
    // {
    //     memcpy(&poseData[ind], &pose[i], sizeof(float));
    //     ind+=sizeof(float);
    // }

    sendData(client_fd, poseData, data_len);
    printf("pose send back\n");
    free(poseData);
    return true;
}

void goToPose(float x0 , float y0, float qx0, float qy0, float qz0, float qw0, 
              float x1 , float y1, float qx1, float qy1, float qz1, float qw1, 
              float x2 , float y2, float qx2, float qy2, float qz2, float qw2){
    printf("go to pose\n");
    printf("set to x0=%f y0=%f x1=%f y1=%f x2=%f y2=%f\n", x0, y0, x1, y1, x2, y2);
    // rotation try
    // pose0
    callForPose(0);
    float originTheta0 = acos(pose[6])*2;
    if(pose[5]<0)
        originTheta0 = -originTheta0;
    float objectTheta0 = acos(qw0)*2;
    if(qz0<0)
        objectTheta0 = -objectTheta0;
    float dTheta0 = objectTheta0 - originTheta0;
    float dx0 = rbt_v * (x0 - pose[0]);
    float dy0 = rbt_v * (y0 - pose[1]);
    // pose1
    callForPose(1);
    float originTheta1 = acos(pose[6])*2;
    if(pose[5]<0)
        originTheta1 = -originTheta1;
    float objectTheta1 = acos(qw1)*2;
    if(qz1<0)
        objectTheta1 = -objectTheta1;
    float dTheta1 = objectTheta1 - originTheta1;
    float dx1 = rbt_v * (x1 - pose[0]);
    float dy1 = rbt_v * (y1 - pose[1]);
    // pose2
    callForPose(2);
    float originTheta2 = acos(pose[6])*2;
    if(pose[5]<0)
        originTheta2 = -originTheta2;
    float objectTheta2 = acos(qw2)*2;
    if(qz2<0)
        objectTheta2 = -objectTheta2;
    float dTheta2 = objectTheta2 - originTheta2;
    float dx2 = rbt_v * (x2 - pose[0]);
    float dy2 = rbt_v * (y2 - pose[1]);
    
    for (int i = 0; i < times; ++i)
    {
        float ox = 0.0, oy = 0.0, oz = 1.0;
        // pose 0
        callForPose(0);
        float crtTheta0 = acos(pose[6])*2;
        if(pose[5]<0)
            crtTheta0 = -crtTheta0;
        float temp_theta0 = crtTheta0 + dTheta0/times;
        setForPose(0, pose[0], pose[1], camera_height, ox*sin(temp_theta0/2), oy*sin(temp_theta0/2), oz*sin(temp_theta0/2), cos(temp_theta0/2));
        
        //printf("test: time%d q: %f, %f, %f, %f\n",
        // i, ox*sin(temp_theta0/2), oy*sin(temp_theta0/2), oz*sin(temp_theta0/2), cos(temp_theta0/2));
        
        // pose 1
        callForPose(1);
        float crtTheta1 = acos(pose[6])*2;
        if(pose[5]<0)
            crtTheta1 = -crtTheta1;
        float temp_theta1 = crtTheta1 + dTheta1/times;
        setForPose(1, pose[0], pose[1], camera_height, ox*sin(temp_theta1/2), oy*sin(temp_theta1/2), oz*sin(temp_theta1/2), cos(temp_theta1/2));
        // pose 2
        callForPose(2);
        float crtTheta2 = acos(pose[6])*2;
        if(pose[5]<0)
            crtTheta2 = -crtTheta2;
        float temp_theta2 = crtTheta2 + dTheta2/times;
        setForPose(2, pose[0], pose[1], camera_height, ox*sin(temp_theta2/2), oy*sin(temp_theta2/2), oz*sin(temp_theta2/2), cos(temp_theta2/2));
        // send to socket
        ros::Rate rate(1);
        rate.sleep();
        ros::spinOnce();
        //getDepth(client_fd);

        getPose(client_fd);   
        getRGBD(client_fd);
         
    }

    // move
    for (int i = 0; i < times; ++i){
        //0
        callForPose(0);
        float tx0 = pose[0] + dx0;
        float ty0 = pose[1] + dy0;
        //1
        callForPose(1);
        float tx1 = pose[0] + dx1;
        float ty1 = pose[1] + dy1;
        //2
        callForPose(2);
        float tx2 = pose[0] + dx2;
        float ty2 = pose[1] + dy2;
        setForPose(0, tx0, ty0, camera_height, qx0, qy0, qz0, qw0);
        setForPose(1, tx1, ty1, camera_height, qx1, qy1, qz1, qw1);
        setForPose(2, tx2, ty2, camera_height, qx2, qy2, qz2, qw2);
        // send to socket
        ros::Rate rate(1);
        rate.sleep();
        ros::spinOnce();
        //getDepth(client_fd);

        getPose(client_fd);
        getRGBD(client_fd);
        
    }
}

// socket move to views
bool move_to_views(int client_fd){

    printf("\nfunc move_to_views begin\n");

    // receive data 
    int data_len = rbtnum * 7 * sizeof(float);
    char* poseData = (char *)malloc(data_len);
    printf("before recv\n");
    int rcv_len = recvData(client_fd, poseData, data_len); // recv
    printf("after recv\n");
    float pass_pose[rbtnum][7];
    int ind = 0;

    // move: set poses to views
    for (int id = 0; id < rbtnum; ++id)
    {
        float set_pose[7];
        for (int i = 0; i < 7; ++i)
        {
            memcpy(&set_pose[i], &poseData[ind], sizeof(float));
            memcpy(&pass_pose[id][i], &poseData[ind], sizeof(float));
            ind+=sizeof(float);
        }
        setForPose(id, 
            set_pose[0], set_pose[1], set_pose[2], 
            set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
        printf("pose for robot%d: %f, %f, %f, %f, %f, %f, %f\n", id, 
            set_pose[0], set_pose[1], set_pose[2],
            set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
    }

    // free 
    free(poseData);

    printf("func move_to_views end\n\n");

    return true;
}

// socket set pose
bool setPose(int client_fd){
    int data_len = rbtnum * 7 * sizeof(float);
    char* poseData = (char *)malloc(data_len);
    printf("before recv\n");
    int rcv_len = recvData(client_fd, poseData, data_len);
    // if(rcv_len!=data_len)
    // 	rcv_len = recvData(client_fd, poseData, data_len);
    printf("after recv\n");
    float pass_pose[rbtnum][7];
    int ind = 0;
    for (int id = 0; id < rbtnum; ++id)
    {
        float set_pose[7];
        for (int i = 0; i < 7; ++i)
        {
            memcpy(&set_pose[i], &poseData[ind], sizeof(float));
            memcpy(&pass_pose[id][i], &poseData[ind], sizeof(float));
            ind+=sizeof(float);
        }
        // setForPose(id, 
        //     set_pose[0], set_pose[1], set_pose[2], 
        //     set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
        // printf("client pose for robot%d: %f, %f, %f, %f, %f, %f, %fn", id, 
        //     set_pose[0], set_pose[1], set_pose[2],
        //     set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
    }
    goToPose(
        pass_pose[0][0], pass_pose[0][1], pass_pose[0][3], pass_pose[0][4], pass_pose[0][5], pass_pose[0][6],
        pass_pose[1][0], pass_pose[1][1], pass_pose[1][3], pass_pose[1][4], pass_pose[1][5], pass_pose[1][6],
        pass_pose[2][0], pass_pose[2][1], pass_pose[2][3], pass_pose[2][4], pass_pose[2][5], pass_pose[2][6]);
    // write to offline
    ofs_off<<pass_pose[0][0]<<" "<<pass_pose[0][1]<<" "<<1<<" "
    << pass_pose[0][3]<<" "<<pass_pose[0][4]<<" "<<pass_pose[0][5]<<" "<<pass_pose[0][6]<<endl;
    ofs_off<<pass_pose[1][0]<<" "<<pass_pose[1][1]<<" "<<1<<" "
    << pass_pose[1][3]<<" "<<pass_pose[1][4]<<" "<<pass_pose[1][5]<<" "<<pass_pose[1][6]<<endl;
    ofs_off<<pass_pose[2][0]<<" "<<pass_pose[2][1]<<" "<<1<<" "
    << pass_pose[2][3]<<" "<<pass_pose[2][4]<<" "<<pass_pose[2][5]<<" "<<pass_pose[2][6]<<endl;



    free(poseData);
    printf("seted pose\n");
    return true;
}

//vector< vector< vector<float> > > send_poses;
struct PositionInGazebo{
	float x;
	float y;
	float z;
	PositionInGazebo(float ix, float iy, float iz){
		x = ix;
		y = iy;
		z = iz;
	}
};
vector< vector< PositionInGazebo > > path_poses;
vector< PositionInGazebo > task_poses;

// set path
bool setPath(int client_fd){
	// clear and resize path_poses
	path_poses.clear();
	path_poses.resize(rbtnum);
	// recv path data
	for (int rid = 0; rid < rbtnum; ++rid)
	{	
		char len_data[sizeof(int)];
		recvData(client_fd, len_data, sizeof(int));//recv data lenth
		int data_len = 0;
		memcpy(&data_len, &len_data, sizeof(int));
		char path_data[MAXRECV];
		recvData(client_fd, path_data, data_len);// recv path data
		// memcpy to path_poses
		int path_size = ceil(data_len/3/sizeof(float));
		int ind = 0;
		for (int pid = 0; pid < path_size; ++pid)
		{
			float x, y, z;
			memcpy(&x, &path_data[ind], sizeof(float));
			ind+=sizeof(float);
			memcpy(&y, &path_data[ind], sizeof(float));
			ind+=sizeof(float);
			memcpy(&z, &path_data[ind], sizeof(float));
			ind+=sizeof(float);
			PositionInGazebo crt_p = PositionInGazebo(x, y, z);
			path_poses[rid].push_back(crt_p);
		}			
	}
	// compute path distance for each robot
	vector<float> path_dises;
	double min_dis = DBL_MAX;
	int min_rid = -1;
	for (int rid = 0; rid < rbtnum; ++rid)
	{
		if(path_poses[rid].size() == 1){ // if dont have move path
			path_dises.push_back(0);
			continue;
		}
		float path_dis = 0;
		for (int pid = 0; pid < path_poses[rid].size()-1; ++pid)
		{
			path_dis+=
			sqrt(
			(path_poses[rid][pid].x-path_poses[rid][pid+1].x)*(path_poses[rid][pid].x-path_poses[rid][pid+1].x)
			+
			(path_poses[rid][pid].y-path_poses[rid][pid+1].y)*(path_poses[rid][pid].y-path_poses[rid][pid+1].y));
		}
		path_dises.push_back(path_dis);
	}
	// compute min distance path
	for (int rid = 0; rid < path_dises.size(); ++rid)
	{
		printf("rbt%d path distance = %f\n", rid, path_dises[rid]);
		if(min_dis>path_dises[rid]){
			min_dis = path_dises[rid];
			min_rid = rid;
		}
	}
	printf("min_dis = %f\n", min_dis);
    {//move_distance_rate
        min_rid = -1;
        min_dis *= move_distance_rate;
    }
	// move robots
	for (int rid = 0; rid < rbtnum; ++rid)
	{
		// move for no path robot
		if(path_poses[rid].size() == 1){
			
			// callForPose(rid);
			// float dir_theta = PI/3;
   //      	// set pose
   //      	setForPose(rid, , , camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			// printf("rbt%d dont move\n", rid);
			// printf("rbt%d set to (%f, %f)\n", rid, pose[0], pose[1]);
			
			callForPose(rid);
			float obj_x=.1, obj_y=.1, crt_x = .1, crt_y = .1;
			obj_x = task_poses[rid].x;
	 		obj_y = task_poses[rid].y;
	 		crt_x = pose[0];
	 		crt_y = pose[1];
	 		float dir_theta;
	 		printf("fabs(obj_x - crt_x)=%f fabs(obj_y - crt_y)=%f\n", fabs(obj_x - crt_x), fabs(obj_y - crt_y));
	 		if(fabs(obj_x - crt_x) < 0.2 && fabs(obj_y - crt_y) < 0.2){ // task_pose == rbt pose
	 			float originTheta = acos(pose[6])*2;
		    	if(pose[5]<0)
		        	originTheta = -originTheta;
		        dir_theta = originTheta+PI/3;
		        if(dir_theta>PI)
		        	dir_theta = -PI + dir_theta - PI;
		        printf("turn PI/3\n");
	 		}else{// task_pose != rbt pose
				// compute direction
				float dx = obj_x - crt_x;
				float dy = obj_y - crt_y;
				// compute rotation
				if (dx > 0){
		          if(dy == 0){
		            dir_theta = 0;
		          }
		          else{
		            dir_theta = atan(dy / dx);
		          }
		        }
		        else if (dx == 0){
		          if(dy > 0){
		            dir_theta = PI/2;
		          }
		          else if (dy == 0){
		            dir_theta = 0; // need to modify to origin direction
		          }
		          else{// dy < 0
		            dir_theta = -PI/2;
		          }
		        }
		        else{ // dx < 0
		          if(dy>0){
		            dir_theta = PI - fabs(atan(dy / dx));
		          }
		          else if(dy == 0){
		            dir_theta = PI;
		          }
		          else{// dy < 0
		            dir_theta = -(PI - fabs(atan(dy / dx)));
		          }
		        }
	 		}
	 		
	        setForPose(rid, crt_x, crt_y, camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			printf("rbt%d dont move\n", rid);
			printf("rbt%d set to (%f, %f)\n", rid, crt_x, crt_y);

			continue;
		}
		// move for min robot
		if(rid == min_rid){
			int node_num = path_poses[rid].size();
			// compute direction
			float dx = task_poses[rid].x - path_poses[rid][node_num-1].x;
			float dy = task_poses[rid].y - path_poses[rid][node_num-1].y;
			// float dx = path_poses[rid][node_num-1].x - path_poses[rid][node_num-2].x;
			// float dy = path_poses[rid][node_num-1].y - path_poses[rid][node_num-2].y;
			float dir_theta;
			// compute rotation
			if (dx > 0){
	          if(dy == 0){
	            dir_theta = 0;
	          }
	          else{
	            dir_theta = atan(dy / dx);
	          }
	        }
	        else if (dx == 0){
	          if(dy > 0){
	            dir_theta = PI/2;
	          }
	          else if (dy == 0){
	            dir_theta = 0; // need to modify to origin direction
	          }
	          else{// dy < 0
	            dir_theta = -PI/2;
	          }
	        }
	        else{ // dx < 0
	          if(dy>0){
	            dir_theta = PI - fabs(atan(dy / dx));
	          }
	          else if(dy == 0){
	            dir_theta = PI;
	          }
	          else{// dy < 0
	            dir_theta = -(PI - fabs(atan(dy / dx)));
	          }
	        }
	        // set pose
        	setForPose(rid, path_poses[rid][node_num-1].x, path_poses[rid][node_num-1].y, camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			printf("rbt%d min\n", rid);
			printf("rbt%d set to (%f, %f)\n", rid, path_poses[rid][node_num-1].x, path_poses[rid][node_num-1].y);
		}
		// move for not min robot
		else{
			float obj_x=.1, obj_y=.1, crt_x = .1, crt_y = .1, lst_x=.1, lst_y=.1;
			float crt_dis = 0;
			for (int pid = 0; pid < path_poses[rid].size()-1; ++pid) // find obj position and lst position
			{
			 	float tmp_dis = 
				 	sqrt(
					(path_poses[rid][pid].x-path_poses[rid][pid+1].x)*(path_poses[rid][pid].x-path_poses[rid][pid+1].x)
					+
					(path_poses[rid][pid].y-path_poses[rid][pid+1].y)*(path_poses[rid][pid].y-path_poses[rid][pid+1].y));
			 	crt_dis += tmp_dis;
			 	//printf("	rbt%d crt_dis = %f\n", rid, crt_dis);
			 	if(crt_dis == min_dis){
			 		//printf("	rbt%d crt_dis == min_dis\n", rid);
			 		obj_x = task_poses[rid].x;
			 		obj_y = task_poses[rid].y;
			 		crt_x = path_poses[rid][pid+1].x;
			 		crt_y = path_poses[rid][pid+1].y;
			 		// obj_x = path_poses[rid][pid+1].x;
			 		// obj_y = path_poses[rid][pid+1].y;
			 		// lst_x = path_poses[rid][pid].x;
			 		// lst_y = path_poses[rid][pid].y;
			 		break;
			 	}
			 	if(crt_dis > min_dis){
			 		//printf("	rbt%d crt_dis > min_dis\n", rid);
			 		crt_dis -= tmp_dis;
			 		float res_dis = min_dis - crt_dis;
			 		{// temp use this
			 			obj_x = task_poses[rid].x;
				 		obj_y = task_poses[rid].y;
				 		crt_x = path_poses[rid][pid+1].x;
				 		crt_y = path_poses[rid][pid+1].y;
			 			// obj_x = path_poses[rid][pid+1].x;
				 		// obj_y = path_poses[rid][pid+1].y;
				 		// lst_x = path_poses[rid][pid].x;
				 		// lst_y = path_poses[rid][pid].y;
			 		}
		 			{/* dai shi xian */
			 			
		 			}

			 		break;
			 	}
			} 
			// compute direction
			float dx = obj_x - crt_x;
			float dy = obj_y - crt_y;
			float dir_theta;
			// compute rotation
			if (dx > 0){
	          if(dy == 0){
	            dir_theta = 0;
	          }
	          else{
	            dir_theta = atan(dy / dx);
	          }
	        }
	        else if (dx == 0){
	          if(dy > 0){
	            dir_theta = PI/2;
	          }
	          else if (dy == 0){
	            dir_theta = 0; // need to modify to origin direction
	          }
	          else{// dy < 0
	            dir_theta = -PI/2;
	          }
	        }
	        else{ // dx < 0
	          if(dy>0){
	            dir_theta = PI - fabs(atan(dy / dx));
	          }
	          else if(dy == 0){
	            dir_theta = PI;
	          }
	          else{// dy < 0
	            dir_theta = -(PI - fabs(atan(dy / dx)));
	          }
	        }
	        // set pose
        	setForPose(rid, crt_x, crt_y, camera_height, 0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
			printf("rbt%d not min\n", rid);
			printf("rbt%d set to (%f, %f)\n", rid, crt_x, crt_y);
		}
	}
	// get scans // send to socket
    ros::Rate rate(1);
    rate.sleep();
    ros::spinOnce();
    getPose(client_fd);
    getRGBD(client_fd);

	//getchar();

	printf("move correctly\n");
	return true;
}

// set up
bool scanSurroundings(int client_fd){

	//float cover_angle = PI/3;
	for (int i = 0; i < 6; ++i)
	{
		for (int rid = 0; rid < rbtnum; ++rid)
		{
			callForPose(rid);
			float originTheta = acos(pose[6])*2;
	    	if(pose[5]<0)
	        	originTheta = -originTheta;
	        float dir_theta = originTheta+PI/3;
	        if(dir_theta>PI)
	        	dir_theta = -PI + dir_theta - PI;
	        setForPose(rid, pose[0], pose[1], camera_height, 
	        	0.0*sin(dir_theta/2), 0.0*sin(dir_theta/2), 1.0*sin(dir_theta/2), cos(dir_theta/2));
		}
		// get scans // send to socket
    	ros::Rate rate(1);
    	rate.sleep();
  	  	ros::spinOnce();
        getPose(client_fd);
    	getRGBD(client_fd);
   	 	
	}	

	return true;
}

// set task positions
bool setTaskPositions(int client_fd){
	task_poses.clear();

	// recv data
	int data_len = rbtnum*2*sizeof(float);
	char otp_data[MAXRECV];
	recvData(client_fd, otp_data, data_len);// recv otp data

	// memcpy to task_poses
	int ind = 0;
	for (int rid = 0; rid < rbtnum; ++rid)
	{
		float x, y;
		memcpy(&x, &otp_data[ind], sizeof(float));
		ind+=sizeof(float);
		memcpy(&y, &otp_data[ind], sizeof(float));
		ind+=sizeof(float);

		PositionInGazebo crt_p = PositionInGazebo(x, y, camera_height);
		task_poses.push_back(crt_p);
	}			

	// {//test
	// 	for (int rid = 0; rid < rbtnum; ++rid)
	// 	{
	// 		printf("task position = (%f, %f)\n", task_poses[rid].x, task_poses[rid].y);
	// 	}
	// }

	return true;
}

void *thread(void *ptr)
{
    //unsigned int tid = (unsigned int)pthread_self(); //获取当前线程id
    int client = *(int *)ptr;
    bool stopped=false;

    while(!stopped){

        char message [MAXRECV+1];

        if(recvData(client_fd, message, MAXRECV+1))
        {
            if(message!=NULL&&strlen(message)!=0)
            {
                printf("message[0]: %c\n", message[0]);
                switch(message[0])
                {
                    case '0':
                    {
                        printf("ask for depth\n");
                        getDepth(client_fd);
                        break;
                    }
                    case '1':
                    {      
                        printf("ask for pose\n");
                        getPose(client_fd);                      
                        break;
                    }
                    case '2':
                    {
                        printf("ask to set pose\n");
                        setPose(client_fd);
                        break;
                    }
                    case '3':
                    {
                        printf("ask for rgbd\n");
                        getRGBD(client_fd);
                        break;
                    }
                    case '4':
                    {
                    	printf("ask to set path\n");
                    	setPath(client_fd);
                    	break;
                    }
                    case '5':
                    {
                    	printf("set up surroundings\n");
                    	scanSurroundings(client_fd);
                    	break;
                    }
                    case '6':
                    {
                    	printf("ask to set task positions\n");
                    	setTaskPositions(client_fd);
                    	break;
                    }
                    case 'm':
                    {
                        printf("move_to_views\n");
                        move_to_views(client_fd);
                        break;
                    }
                   
                }
            }
        }
        usleep(100000);
    }
    return 0;
}

int main(int argc, char **argv){

    // run move_controller
    

    ros::init(argc, argv, "octo_navi");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    // calibration
    //ros::Subscriber calib_sub = n.subscribe("camera/depth/camera_info", 1, infoCallback);
    // write
    ofs_off.open("/home/dsy/catkin_ws/src/virtual_scan/data/offline/pose_history.txt");
    //ofs_off.close();
    // topic: image
    image_transport::Subscriber sub0rgb = it.subscribe("camera0/rgb/image_raw", 1, rgbImageCallback0);
    image_transport::Subscriber sub0depth = it.subscribe("camera0/depth/image_raw", 1, depthImageCallback0);
    image_transport::Subscriber sub1rgb = it.subscribe("camera1/rgb/image_raw", 1, rgbImageCallback1);
    image_transport::Subscriber sub1depth = it.subscribe("camera1/depth/image_raw", 1, depthImageCallback1);
    image_transport::Subscriber sub2rgb = it.subscribe("camera2/rgb/image_raw", 1, rgbImageCallback2);
    image_transport::Subscriber sub2depth = it.subscribe("camera2/depth/image_raw", 1, depthImageCallback2);
    image_transport::Subscriber sub3rgb = it.subscribe("camera3/rgb/image_raw", 1, rgbImageCallback3);
    image_transport::Subscriber sub3depth = it.subscribe("camera3/depth/image_raw", 1, depthImageCallback3);
    image_transport::Subscriber sub4rgb = it.subscribe("camera4/rgb/image_raw", 1, rgbImageCallback4);
    image_transport::Subscriber sub4depth = it.subscribe("camera4/depth/image_raw", 1, depthImageCallback4);
    // service: pose
    gclient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    sclient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

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

        pthread_t id;
        int ret = pthread_create(&id, NULL, thread, &client_fd);
        if(ret) 
        {
            printf("Create pthread error!\n");
        }

        usleep(100000);
    }

    // // collect info offline
    // ofstream pose_ofs;
    // pose_ofs.open("src/virtual_scan/data/pose.txt");
    // int count = 0;
    // char key = 'x';
    // while(1){
    //     // // multi robots
    //     // if(key=='0'){
    //     //     shortImg = shortImg0;
    //     // }
    //     // else if(key=='1'){
    //     //     shortImg = shortImg1;
    //     // }
    //     // else if(key=='2'){
    //     //     shortImg = shortImg2;
    //     // }
    //     // else if(key=='e'){
    //     //     break;
    //     // }
    //     // key = 'x';
    //     // ros::spinOnce();
    //     // cv::imshow("short", shortImg);
    //     // key = cv::waitKey(1);
        
    //     // collect info
    //     ros::spinOnce();
    //     callForPose();
    //     shortImg = shortImg0;
    //     if(key=='a'){
    //         //ros::spinOnce();
    //         //callForPose();
    //         //shortImg = shortImg0;
    //         if(depth_ready && pose_ready){
    //             printf("ready\n");
    //             // write img and pose
    //             stringstream ss;
    //             string s;
    //             ss<<"src/virtual_scan/data/depth"<<count<<".png";
    //             ss>>s;
    //             cv::imwrite(s, shortImg);
    //             pose_ofs<<pose[0]<<" "
    //             <<pose[1]<<" "
    //             <<pose[2]<<" "
    //             <<pose[3]<<" "
    //             <<pose[4]<<" "
    //             <<pose[5]<<" "
    //             <<pose[6]<<" "
    //             <<endl;
    //             depth_ready = false;
    //             pose_ready = false;
    //             count++;
    //         }
    //     }
    //     else if(key=='e')
    //         break;
    //     key = 'x';
    //     ros::spinOnce();
    //     //cv::imshow("rgb", rgbImg);
    //     //cv::waitKey(1);
    //     cv::imshow("short", shortImg);
    //     key = cv::waitKey(1);
    // }
    // pose_ofs.close();

    return 0;
}