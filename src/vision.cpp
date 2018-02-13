/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Orange Team in VVV18
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "vision.h"

Processing::Processing( const std::string &moduleName )
{
    this->moduleName = moduleName;
}

/********************************************************/
Processing::~Processing()
{

};

/********************************************************/
bool Processing::open(){

    this->useCallback();

    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
    inPort.open("/"+ moduleName + "/image:i");
    outPort.open("/"+ moduleName + "/image:o");
    cropOutPort.open("/" + moduleName + "/crop:o");
    targetPort.open("/"+ moduleName + "/target:o");

    return true;
}

/********************************************************/
void Processing::close()
{
    inPort.close();
    outPort.close();
    targetPort.close();
    cropOutPort.close();
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::close();
}

/********************************************************/
void Processing::interrupt()
{
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::interrupt();
}

/********************************************************/
void Processing::onRead( yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = outPort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &cropOutImage  = cropOutPort.prepare();
    yarp::os::Bottle &outTargets = targetPort.prepare();
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();

    outImage.resize(dispImage.width(), dispImage.height());
    cropOutImage.resize(dispImage.width(), dispImage.height());
    
    outImage.zero();
    cropOutImage.zero();
    
    cv::Mat inColour_cv = cv::cvarrToMat((IplImage *)inImage->getIplImage());  // prepare the image ports and targets
    cv::Mat disp = cv::cvarrToMat((IplImage *)dispImage.getIplImage());
    
    // Apply image processing techniques on the disparity image to smooth things out 
    int gausian_size = 9;
    cv::GaussianBlur(disp, disp, cv::Size(gausian_size, gausian_size), 2, 2);

    // Apply some threshold on the image to remove background:
    // have a look at cv::threshold function
    // cv.Threshold(src, dst, threshold, maxValue, thresholdType)
    cv::threshold(disp, disp, 80, 255, cv::THRESH_TOZERO);

    // Find the max value and its position
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    minMaxLoc(disp, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    //double x_pos = maxLoc.x;
    //double y_pos = maxLoc.y;
    cv::Point2f test_point = cv::Point2f(maxLoc.x, maxLoc.y);

    //Find the contour of the closest objects with moments and mass center
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(disp, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

    // optional hint: you could use pointPolygonTest and the previous maxvalue location to compare with all contours found and get the actual brightest one

    // Use the result of pointPolygonTest or your own technique as the closest contour to:
    cv::Scalar color(255, 255, 0);
    //cv::drawContours(disp, contours, -1, color, 2, 8);
    for(size_t i = 0; i < contours.size(); i++) {
        if(pointPolygonTest(contours[i], test_point, false) != -1) {
            // 1 - draw it on the disparity image
            cv::drawContours(disp, contours, i, color, 2, 8);

            // 2 - create a cropped image containing the rgb roi
            cv::Rect rect = cv::boundingRect(contours[i]);
            cv::rectangle(disp, rect.tl(), rect.br(), color, 2, 8, 0);

            cv::rectangle(inColour_cv, rect.tl(), rect.br(), color, 2, 8, 0);

            /*
            cv::Mat roi = inColour_cv(rect);
            inColour_cv.convertTo(inColour_cv, CV_8UC3, 0.5);
            roi.copyTo(inColour_cv(rect));
            */

            // 3 - fill in a yarp bottle with the bounding box
            //be aware that the expected Bottle should be a list containing:
            // (tl.x tl.y br.x br.y)
            //where tl is top left and br - bottom right
            outTargets.clear();
            yarp::os::Bottle &tmp = outTargets.addList();

            tmp.addInt(rect.tl().x);
            tmp.addInt(rect.tl().y);
            tmp.addInt(rect.br().x);
            tmp.addInt(rect.br().y);
        }
    }

    cvtColor(disp, disp, CV_GRAY2RGB);
    
    if (outTargets.size() > 0)
        targetPort.write();          

    IplImage out = disp;
    outImage.resize(out.width, out.height);
    cvCopy( &out, (IplImage *) outImage.getIplImage());
    outPort.write();

    IplImage crop = inColour_cv;
    cropOutImage.resize(crop.width, crop.height);
    cvCopy( &crop, (IplImage *) cropOutImage.getIplImage());
    cropOutPort.write();
}



bool Module::configure(yarp::os::ResourceFinder &rf)
{
    this->rf=&rf;
    std::string moduleName = rf.check("name", yarp::os::Value("closest-blob"), "module name (string)").asString();
    setName(moduleName.c_str());

    rpcPort.open(("/"+getName("/rpc")).c_str());

    closing = false;

    processing = new Processing( moduleName );

    /* now start the thread to do the work */
    processing->open();

    attach(rpcPort);

    return true;
}

/**********************************************************/
bool Module::close()
{
    processing->interrupt();
    processing->close();
    delete processing;
    return true;
}

/**********************************************************/
bool Module::quit(){
    closing = true;
    return true;
}

/********************************************************/
double Module::getPeriod()
{
    return 0.1;
}

/********************************************************/
bool Module::updateModule()
{
    return !closing;
}

