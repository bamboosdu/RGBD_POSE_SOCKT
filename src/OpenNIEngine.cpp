#include <iostream>
#include "OpenNIEngine.h"

using namespace std;

OpenNIEngine::OpenNIEngine(){
    openCamera();

    oniDepthStream.readFrame(&oniDepthImg);
    oniColorStream.readFrame(&oniColorImg);

    image_height = oniColorImg.getHeight();
    image_width = oniColorImg.getWidth();

    rgbd = (char *)malloc(image_height * image_width * (sizeof(unsigned char)*3 + sizeof(short)));
    memset(rgbd, 0, image_height * image_width * (sizeof(unsigned char)*3 + sizeof(short)));

//    r = (uchar *)malloc(image_height * image_width * sizeof(unsigned char));
//    g = (uchar *)malloc(image_height * image_width * sizeof(unsigned char));
//    b = (uchar *)malloc(image_height * image_width * sizeof(unsigned char));
//    depth = (short *)malloc(image_height * image_width * sizeof(short));
      
      r = (uchar *)rgbd;
      g = (uchar *)(rgbd + image_height * image_width);
      b = (uchar *)(rgbd + image_height * image_width * 2);
      depth = (short *)(rgbd + image_height * image_width * 3);
}

OpenNIEngine::~OpenNIEngine(){
    closeCamera();

    if(rgbd != NULL){
       free(rgbd);
       rgbd = NULL;
    }

//    if(r != NULL){
//       free(r);
//       r = NULL;
//    }

//    if(g != NULL){
//       free(g);
//       g = NULL;
//    }

//    if(b != NULL){
//       free(b);
//       b = NULL;
//    }

//    if(depth != NULL){
//       free(depth);
//       depth = NULL;
//    }
}

bool OpenNIEngine::openCamera(){
    Status rc = STATUS_OK;

    //initialize OpenNI2
    rc = OpenNI::initialize();

    //open kinect or xtion
    const char * deviceURL = openni::ANY_DEVICE;
    rc = device.open(deviceURL);

    //create depth stream
    rc = oniDepthStream.create(device, SENSOR_DEPTH);
    if (STATUS_OK == rc)
    {
        //set depth video mode
        VideoMode modeDepth;
        modeDepth.setResolution(640, 480);
        modeDepth.setFps(30);
        modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
        oniDepthStream.setVideoMode(modeDepth);
    }
    else
    {
        cerr << "Can't create depth stream: " << OpenNI::getExtendedError() << endl;
        return false;
    }

    //create color stream
    rc = oniColorStream.create(device, openni::SENSOR_COLOR);
    if (STATUS_OK == rc)
    {
        //set color video mode
        VideoMode modeColor;
        modeColor.setResolution(640, 480);
        modeColor.setFps(30);
        modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
        oniColorStream.setVideoMode(modeColor);
    }
    else
    {
        cerr << "Can't create color stream: " << OpenNI::getExtendedError() << endl;
        return false;
    }

    if (!oniDepthStream.isValid() || !oniColorStream.isValid())
    {
        cerr << "illegal!" << endl;
        OpenNI::shutdown();
        return false;
    }

    openni::Status s;
    if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        s = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR); //depth to color registration
    }

    rc = oniDepthStream.start(); //open depth stream
    if (STATUS_OK != rc)
    {
        cerr << "Can't open depth stream: " << OpenNI::getExtendedError() << endl;
        oniDepthStream.destroy();

        return false;
    }

    rc = oniColorStream.start(); //open color stream
    if (STATUS_OK != rc)
    {
        cerr << "Can't open color stream: " << OpenNI::getExtendedError() << endl;
        oniColorStream.destroy();

        return false;
    }

    return true;
}

bool OpenNIEngine::closeCamera(){
    oniDepthStream.destroy();
    oniColorStream.destroy();
    device.close();
    OpenNI::shutdown();

    return true;
}


bool OpenNIEngine::hasMoreImages(){
    return true;
}

bool OpenNIEngine::getNewImages(){
    oniDepthStream.readFrame(&oniDepthImg);
    oniColorStream.readFrame(&oniColorImg);

    const int height = oniColorImg.getHeight();
    const int width = oniColorImg.getWidth();

    if ((oniDepthImg.getWidth() != oniColorImg.getWidth()) || (oniDepthImg.getHeight() != oniColorImg.getHeight())){
        cout << endl << "The RGB and the depth frames don't have the same size.";
        return false;
    }
    else
    {
        //Read new frame
        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)oniDepthImg.getData();
        const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)oniColorImg.getData();
        int rowSize = oniColorImg.getStrideInBytes() / sizeof(openni::RGB888Pixel);

        for (int yc = 0; yc < height; yc++)
        {
            const openni::RGB888Pixel* pRgb = pRgbRow;
            const openni::DepthPixel* pDepth = pDepthRow;
            for (int xc = 0; xc < width; xc++, ++pRgb, ++pDepth)
            {
                int ind = yc*width + width -1 - xc;
		depth[ind] = *pDepth;
		r[ind] = pRgb->r;
		g[ind] = pRgb->g;
		b[ind] = pRgb->b;
            }
            pRgbRow += rowSize;
            pDepthRow += rowSize;
        }
    }

    return true;
}

uchar* OpenNIEngine::getR(){
    return r;
}

uchar* OpenNIEngine::getG(){
    return g;
}

uchar* OpenNIEngine::getB(){
    return b;
}

short* OpenNIEngine::getDepth(){
    return depth;
}

int OpenNIEngine::getImageWidth(){
    return image_width;
}

int OpenNIEngine::getImageHeight(){
    return image_height;
}

char* OpenNIEngine::getRGBD(){
    return rgbd;
}



