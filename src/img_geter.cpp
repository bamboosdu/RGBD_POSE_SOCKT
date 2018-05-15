#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
//#include <sstream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <arpa/inet.h>

#include <cstring>

#define RGBLEN 921600
#define DEPTHLEN  307200

using namespace std;
using namespace cv;

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
    printf("rgbImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    Mat rgbImg;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgbImg = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
     // get rows and cols and channels
    //int rows = rgbImg.rows;
    //int cols = rgbImg.cols;
    //int channels = (int)rgbImg.channels();
    //  show img
    try
    {
        imshow("rgb", rgbImg);
        waitKey(1);
    }
    catch (Exception& e)
    {
        ROS_ERROR("rgb exception: %s", e.what());
    }//*/
}
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
    printf("depthImageCallback\n");
    cv_bridge::CvImagePtr cvImgPtr;
    //Mat depthImg(480, 640, CV_32FC1);
    Mat depthImg;
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
    /*// /10
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            printf("%f\n", depthImg.ptr<float>(i)[j]);
            depthImg.ptr<float>(i)[j];
        }
    }//*/
    // short img
    Mat shortImg(480, 640, CV_16UC1);
    //printf("size of short: %lu\n", sizeof(short));
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            //uint8_t
            shortImg.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j] * 1000);
        }
    }
    /*//  show img
    try
    {
        imshow("depth", depthImg);
        waitKey(1);
    }
    catch (Exception& e)
    {
        ROS_ERROR("depth exception: %s", e.what());
    }//*/
    try
    {
        imshow("short", shortImg);
        waitKey(1);
    }
    catch (Exception& e)
    {
        ROS_ERROR("short exception: %s", e.what());
    }
    //imwrite("depth_test.png", shortImg);
    /*sensor_msgs::Image img = *msg;
    float* data = (float*)&img.data[0];
    Mat depthImg;
    printf("ok\n");
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            depthImg.ptr<float>(i)[j] = data[i*640+j];
        }
    }
    printf("ok\n");
    //  show img
    try
    {
        imshow("depth", depthImg);
        waitKey(1);
    }
    catch (Exception& e)
    {
        ROS_ERROR("depth exception: %s", e.what());
    }//*/
}
// main
int main(int argc, char **argv){

    ros::init(argc, argv, "image_geter");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub1 = it.subscribe("camera/rgb/image_raw", 1, rgbImageCallback);
    //image_transport::Subscriber sub2 = it.subscribe("camera/depth_registered/image_raw", 1, depthImageCallback);
    image_transport::Subscriber sub2 = it.subscribe("camera/depth/image_raw", 1, depthImageCallback);
    //image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

    ros::spin();
//*/
    return 0;
}
