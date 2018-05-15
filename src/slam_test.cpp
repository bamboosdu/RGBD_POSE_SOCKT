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

const int rbtnum = 3;

cv::Mat rgbImg;
cv::Mat shortImg(480, 640, CV_16UC1);
bool rgb_ready = false;
bool depth_ready = false;

cv::Mat rgbImg0;
cv::Mat shortImg0(480, 640, CV_16UC1);
cv::Mat rgbImg1;
cv::Mat shortImg1(480, 640, CV_16UC1);
cv::Mat rgbImg2;
cv::Mat shortImg2(480, 640, CV_16UC1);

ros::ServiceClient gclient;
ros::ServiceClient sclient;
float pose[7];
bool pose_ready = false;

const float camera_factor = 1000;
const float camera_cx = 320.500000;
const float camera_cy = 240.500000;
const float camera_fx = 554.382713;
const float camera_fy = 554.382713;

float rbt_v = 0.3;

float camera_height = 1.0;

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
        // rbt0
        int ind = 0;
        for (int i = 0; i < 480; ++i)
        {
            for (int j = 0; j < 640; ++j)
            {
                memcpy(&rgbData[ind], &rgbImg0.ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
                ind+=sizeof(uchar);
                memcpy(&rgbData[ind], &rgbImg0.ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
                ind+=sizeof(uchar);
                memcpy(&rgbData[ind], &rgbImg0.ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
                ind+=sizeof(uchar);
            }
        }
        // rbt1
        for (int i = 0; i < 480; ++i)
        {
            for (int j = 0; j < 640; ++j)
            {
                memcpy(&rgbData[ind], &rgbImg1.ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
                ind+=sizeof(uchar);
                memcpy(&rgbData[ind], &rgbImg1.ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
                ind+=sizeof(uchar);
                memcpy(&rgbData[ind], &rgbImg1.ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
                ind+=sizeof(uchar);
            }
        }
        // rbt2
        for (int i = 0; i < 480; ++i)
        {
            for (int j = 0; j < 640; ++j)
            {
                memcpy(&rgbData[ind], &rgbImg2.ptr<cv::Vec3b>(i)[j][0], sizeof(uchar));
                ind+=sizeof(uchar);
                memcpy(&rgbData[ind], &rgbImg2.ptr<cv::Vec3b>(i)[j][1], sizeof(uchar));
                ind+=sizeof(uchar);
                memcpy(&rgbData[ind], &rgbImg2.ptr<cv::Vec3b>(i)[j][2], sizeof(uchar));
                ind+=sizeof(uchar);
            }
        }
        sendData(client_fd, rgbData, data_len);
        printf("rgb send back\n");
        free(rgbData);
    }   
    // depth
    {
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
    // rbt0
    callForPose(0);
    int ind = 0;
    for (int i = 0; i < 7; ++i)
    {
        memcpy(&poseData[ind], &pose[i], sizeof(float));
        ind+=sizeof(float);
    }
    // rbt1
    callForPose(1);
    for (int i = 0; i < 7; ++i)
    {
        memcpy(&poseData[ind], &pose[i], sizeof(float));
        ind+=sizeof(float);
    }
    // rbt2
    callForPose(2);
    for (int i = 0; i < 7; ++i)
    {
        memcpy(&poseData[ind], &pose[i], sizeof(float));
        ind+=sizeof(float);
    }
    sendData(client_fd, poseData, data_len);
    printf("pose send back\n");
    free(poseData);
    return true;
}

void goToPose(float x0 , float y0, float qx0, float qy0, float qz0, float qw0, 
              float x1 , float y1, float qx1, float qy1, float qz1, float qw1, 
              float x2 , float y2, float qx2, float qy2, float qz2, float qw2){
    
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
    int times = 4;
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
        
        printf("test: time%d q: %f, %f, %f, %f\n",
         i, ox*sin(temp_theta0/2), oy*sin(temp_theta0/2), oz*sin(temp_theta0/2), cos(temp_theta0/2));
        
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
        getDepth(client_fd);
        getPose(client_fd);    
    }

    printf("test: final q: %f, %f, %f, %f\n",
        qx0, qy0, qz0, qw0);

    // // rotation origin
    // // pose 0
    // callForPose(0);
    // setForPose(0, pose[0], pose[1], camera_height, qx0, qy0, qz0, qw0);
    // float dx0 = rbt_v * (x0 - pose[0]);
    // float dy0 = rbt_v * (y0 - pose[1]);
    // // pose 1
    // callForPose(1);
    // setForPose(1, pose[0], pose[1], camera_height, qx1, qy1, qz1, qw1);
    // float dx1 = rbt_v * (x1 - pose[0]);
    // float dy1 = rbt_v * (y1 - pose[1]);
    // // pose 1
    // callForPose(2);
    // setForPose(2, pose[0], pose[1], camera_height, qx2, qy2, qz2, qw2);
    // float dx2 = rbt_v * (x2 - pose[0]);
    // float dy2 = rbt_v * (y2 - pose[1]);
    // // send to socket
    // ros::Rate rate(1);
    // rate.sleep();
    // ros::spinOnce();
    // getDepth(client_fd);
    // getPose(client_fd);

    // while(1){
    //     //0
    //     callForPose(0);
    //     float tx0 = pose[0] + dx0;
    //     float ty0 = pose[1] + dy0;
    //     //1
    //     callForPose(1);
    //     float tx1 = pose[0] + dx1;
    //     float ty1 = pose[1] + dy1;
    //     //2
    //     callForPose(2);
    //     float tx2 = pose[0] + dx2;
    //     float ty2 = pose[1] + dy2;

    //     if(dx0>=0){
    //        if(tx0>=ox0){
    //             tx0 = ox0;
    //             ty0 = oy0;
    //             tx1 = ox1;
    //             ty1 = oy1;
    //             tx2 = ox2;
    //             ty2 = oy2;
    //             break;
    //        }
    //     }else{
    //         if(tx0<=ox0){
    //             tx0 = ox0;
    //             ty0 = oy0;
    //             tx1 = ox1;
    //             ty1 = oy1;
    //             tx2 = ox2;
    //             ty2 = oy2;
    //             break;
    //         }
    //     }
    //     setForPose(0, tx0, ty0, 1, qx0, qy0, qz0, qw0);
    //     setForPose(1, tx1, ty1, 1, qx1, qy1, qz1, qw1);
    //     setForPose(2, tx2, ty2, 1, qx2, qy2, qz2, qw2);

    // }
    // setForPose(0, tx0, ty0, 1, qx0, qy0, qz0, qw0);
    // setForPose(1, tx1, ty1, 1, qx1, qy1, qz1, qw1);
    // setForPose(2, tx2, ty2, 1, qx2, qy2, qz2, qw2);

    // move
    for (int i = 0; i < 3; ++i){
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
        getDepth(client_fd);
        getPose(client_fd);

    }
}

// socket set pose
bool setPose(int client_fd){
    int data_len = rbtnum * 7 * sizeof(float);
    char* poseData = (char *)malloc(data_len);
    recvData(client_fd, poseData, data_len);
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
                   
                }
            }
        }
        usleep(100000);
    }
    return 0;
}

int main(int argc, char **argv){

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
    // service: pose
    gclient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    sclient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // // 0 pose
    // float x0 = 0.0, y0 = -1.0, z0 = camera_height;
    // float theta0 = 0.0;
    // float ox0 = 0.0, oy0 = 0.0, oz0 = 1.0;
    // geometry_msgs::Pose start_pose0;
    //         start_pose0.position.x = x0;
    //         start_pose0.position.y = y0;
    //         start_pose0.position.z = z0;
    //         start_pose0.orientation.x = ox0*sin(theta0/2);
    //         start_pose0.orientation.y = oy0*sin(theta0/2);
    //         start_pose0.orientation.z = oz0*sin(theta0/2);
    //         start_pose0.orientation.w = cos(theta0/2);
    // geometry_msgs::Twist start_twist0;
    //         start_twist0.linear.x = 0.0;
    //         start_twist0.linear.y = 0.0;
    //         start_twist0.linear.z = 0.0;
    //         start_twist0.angular.x = 0.0;
    //         start_twist0.angular.y = 0.0;
    //         start_twist0.angular.z = 0.0;
    // gazebo_msgs::ModelState modelstate0;
    //         modelstate0.model_name = (std::string) "test0";
    //         modelstate0.reference_frame = (std::string) "world";
    //         modelstate0.pose = start_pose0;
    //         modelstate0.twist = start_twist0;
    // gazebo_msgs::SetModelState setmodelstate0;
    // setmodelstate0.request.model_state = modelstate0;
    // //ROS_INFO("call server to move a model");
    // if (sclient.call(setmodelstate0))
    // {
    //     //ROS_INFO("success");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service");
    //     return 1;
    // }
    

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