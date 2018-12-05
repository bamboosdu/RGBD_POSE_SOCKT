#include <stdio.h> 
#include <stdlib.h> 
#include <cstdio>


const int rbtnum = 1;

bool rgb_ready = false;
bool depth_ready = false;


vector<cv::Mat> crt_rgb_images(rbtnum);
vector<cv::Mat> crt_depth_images(rbtnum);

// 0 rgb
void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    cv_bridge::CvImagePtr cvImgPtr;
    cv::Mat rgbImg;
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
    crt_rgb_images[0] = rgbImg;
    rgb_ready = true;
    //  show img
    // printf("show the rgb image.\n");
    // cv::imshow("rgb", rgbImg);
    // getchar();
    //cv::waitKey(1);
    //
}

// 0 depth
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    
    cv_bridge::CvImagePtr cvImgPtr;
    cv::Mat depthImg;
    // get rgbinfo into mat rgbImgImg
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//TYPE_32FC1
        depthImg = cvImgPtr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    cv::Mat shortImg(480, 640, CV_16UC1);
    
    for (int i = 0; i < 480; ++i)
    {
        for (int j = 0; j < 640; ++j)
        {
            
            shortImg.ptr<short>(i)[j] = (short)(depthImg.ptr<float>(i)[j]);//* 1000);
        }
    }
    crt_depth_images[0] = shortImg;
    depth_ready = true;
    //  show img
    //printf("show the depth image.\n");
    //cv::imshow("depth", depthImg);
    //cv::imshow("short", shortImg0);
    //getchar();
   // cv::waitKey(1);
    //*/
}

// socket get rgbd
bool getRGBD(int client_fd){
    //ros::spinOnce();
    // rgb
   
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

        printf("copy the depth image\n");
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
    depth_ready = false;


    return true;
}


