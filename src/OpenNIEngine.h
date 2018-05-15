#pragma once

#include "openni2/OpenNI.h"

using namespace openni;

typedef unsigned char uchar;

class OpenNIEngine
{

public:
    OpenNIEngine();
    ~OpenNIEngine();

    bool hasMoreImages();
    bool getNewImages();

    char *getRGBD();

    uchar *getR();
    uchar *getG();
    uchar *getB();
    short *getDepth();  

    int getImageWidth();
    int getImageHeight();

    bool openCamera();
    bool closeCamera();

private:
    VideoStream oniDepthStream;
    VideoStream oniColorStream;
    Device device;

    VideoFrameRef oniDepthImg;
    VideoFrameRef oniColorImg;

    char *rgbd;//r: image_height * image_width * sizeof(unsigned char) then
               //g: image_height * image_width * sizeof(unsigned char) then 
               //b: image_height * image_width * sizeof(unsigned char) then
               //depth: image_height * image_width * sizeof(unsigned char)

    uchar *r;
    uchar *g;
    uchar *b;
    short *depth;

    int image_width;
    int image_height;
};

