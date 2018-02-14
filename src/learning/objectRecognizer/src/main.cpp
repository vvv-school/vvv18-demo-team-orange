//
// A tutorial on how to wrap Caffe in YARP and recognize objects in images.
//
// Author: Giulia Pasquale - <giulia.pasquale@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/Stamp.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <highgui.h>
#include <cv.h>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <cstdio>
#include <cstdlib> // getenv
#include <string>
#include <deque>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>

#include "CaffeWrapper.hpp"
#include "definitions.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class ObjectRecognizerPort: public BufferedPort<Image>
{
private:

    // Resource Finder and module options

    Semaphore              mutex;

    cv::Mat                img_mat;
    cv::Mat                img_crop_mat;

    int                    radius;
    int                    crop_mode;

    CaffeWrapper<float>    *caffe_wrapper;

    BufferedPort<Bottle>   port_in_centroid;
    BufferedPort<Bottle>   port_in_roi;

    Port                   port_out_view;
    Port                   port_out_scores;
    Port                   port_out_hist;

    vector<cv::Scalar>     colors;

    string*                labels;
    int                    n_classes;

    void onRead(Image &img)
    {

        mutex.wait();

        // If something arrived...
        if (img.width()>0 && img.height()>0)
        {

            // convert from RGB to BGR
            img_mat = cv::cvarrToMat((IplImage*)img.getIplImage());
            cv::cvtColor(img_mat, img_mat, CV_RGB2BGR);

            // extract the crop: init variables
            bool crop_found = false;
            bool crop_valid = false;
            int x=-1;
            int y=-1;
            int pixelCount=0;
            int tlx = -1;
            int tly = -1;
            int brx  = -1;
            int bry = -1;

            switch (crop_mode)
            {
                case FIXED:
                {
                    x = floor(img_mat.cols*0.5f);
                    y = floor(img_mat.rows*0.5f);
                    crop_found = true;
                } break;
                case CENTROID:
                {
                    Bottle *centroid = port_in_centroid.read(true);
                    if (centroid!=NULL)
                    {
                        Bottle *window = centroid->get(0).asList();
                        x = window->get(0).asInt();
                        y = window->get(1).asInt();
                        pixelCount = window->get(2).asInt();
                        crop_found = true;
                    }

                } break;
                case ROI:
                {
                    Bottle *roi = port_in_roi.read(false);
                    if (roi!=NULL)
                    {
                        Bottle *window = roi->get(0).asList();
                        tlx = window->get(0).asInt();
                        tly = window->get(1).asInt();
                        brx = window->get(2).asInt();
                        bry = window->get(3).asInt();
                        crop_found = true;
                    }
                } break;
                default:
                {
                    std::cout<< "Non valid crop_mode!" << std::endl;
                    mutex.post();
                    return;
                }
            }

            // extract the crop: validate the coordinates
            if (crop_found)
            {
                switch(crop_mode)
                {
                    case FIXED:
                    case CENTROID:
                    {
                        int r = std::min(radius,x);
                        r = std::min(r,y);
                        r = std::min(r,img_mat.cols-x-1);
                        r = std::min(r,img_mat.rows-y-1);
                        if (r>10)
                        {
                            tlx = x-r;
                            tly = y-r;
                            brx = x+r;
                            bry = y+r;
                            crop_valid = true;
                        }
                    } break;
                    case ROI:
                    {
                        tlx = std::max(tlx, 0);
                        tly = std::max(tly, 0);
                        brx = std::max(brx, 0);
                        bry = std::max(bry, 0);
                        tlx = std::min(tlx, img_mat.cols);
                        tly = std::min(tly, img_mat.rows);
                        brx = std::min(brx, img_mat.cols);
                        bry = std::min(bry, img_mat.rows);
                        if (brx-tlx>20 && bry-tly>20)
                        crop_valid = true;
                    } break;
                    default:
                    {
                        std::cout<< "Non valid crop_mode!" << std::endl;
                        mutex.post();
                        return;
                    }
                }

                // extract the crop: do it
                if (crop_valid)
                {
                    // crop the image
                    cv::Rect img_ROI = cv::Rect(cv::Point( tlx, tly ), cv::Point( brx, bry ));
                    img_crop_mat.resize(img_ROI.width, img_ROI.height);
                    img_mat(img_ROI).copyTo(img_crop_mat);

                    // extract the scores
                    std::vector<float> scores;
                    if (!caffe_wrapper->forward(img_crop_mat, scores))
                    {
                        std::cout << "forward(): failed..." << std::endl;
                        mutex.post();
                        return;
                    }
                    if (scores.size()!=n_classes)
                    {
                        std::cout << n_classes << std::endl;
                        std::cout << scores.size() << std::endl;
                        std::cout << "number of labels differs from number of scores!" << std::endl;
                        mutex.post();
                        return;
                    }

                    // compute max of scores
                    int max_idx = 0;
                    float max_score = scores[max_idx];

                    for (int class_idx=1; class_idx<n_classes; class_idx++)
                    {
                        if (scores[class_idx] > max_score)
                        {
                            max_idx = class_idx;
                            max_score = scores[max_idx];
                        }
                    }

                    // print the scores
                    std::cout << "SCORES: " << endl;
                    for (int i=0; i<n_classes; i++)
                    std::cout << "[" << labels[i] << "]: " << scores[i] << std::endl;
                    std::cout << std::endl << std::endl;

                    // prepare outputs
                    Stamp stamp;
                    this->getEnvelope(stamp);

                    // send out the histogram
                    if (port_out_hist.getOutputCount()>0)
                    {
                        // init dims
                        int img_hist_height = 600;
                        int img_hist_width = 800;
                        ImageOf<PixelRgb> img_hist;
                        img_hist.resize(img_hist_width,img_hist_height);
                        img_hist.zero();

                        int bin_width = img_hist.width()/n_classes;
                        int bin_bottom = img_hist_height;

                        // int auxiliary images
                        cv::Mat img_hist_mat = cv::cvarrToMat(img_hist.getIplImage());
                        cv::Mat img_text_mat = cv::Mat::zeros(img_hist.width(), img_hist.height(), CV_8UC3);

                        // draw
                        for (int bin_idx=0; bin_idx<n_classes; bin_idx++)
                        {
                            int bin_top = (int)(img_hist_height*(1.0f - scores[bin_idx]));
                            cv::rectangle(img_hist_mat, cv::Point(bin_idx*bin_width,bin_top),
                            cv::Point((bin_idx+1)*bin_width,bin_bottom),
                            colors[bin_idx%(int)colors.size()],CV_FILLED);
                        }
                        for (int bin_idx=0; bin_idx<n_classes; bin_idx++)
                        {
                            cv::putText(img_text_mat,labels[bin_idx].c_str(),
                            cv::Point(img_hist_height - bin_bottom, bin_idx*bin_width+bin_width/2),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
                        }
                        transpose(img_text_mat, img_text_mat);
                        flip(img_text_mat, img_text_mat, 0);
                        img_hist_mat = img_hist_mat + img_text_mat;

                        port_out_hist.write(img_hist);

                    }

                    // send out the scores
                    if (port_out_scores.getOutputCount())
                    {
                        Bottle scores_bottle;
                        for (int i=0; i<scores.size(); i++)
                        {
                            Bottle &b = scores_bottle.addList();
                            b.addString(labels[i].c_str());
                            b.addDouble(scores[i]);
                        }
                        port_out_scores.write(scores_bottle);
                    }

                    // send out the predicted label over the cropped region
                    if (port_out_view.getOutputCount())
                    {
                        int y_text, x_text;
                        y_text = tly-10;
                        x_text = tlx;
                        if (y_text<5)
                        y_text = bry+2;

                        cv::cvtColor(img_mat, img_mat, CV_RGB2BGR);
                        cv::rectangle(img_mat,cv::Point(tlx,tly),cv::Point(brx,bry),cv::Scalar(0,255,0),2);
                        cv::putText(img_mat,labels[max_idx].c_str(),cv::Point(x_text,y_text), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,255,0), 4);

                        port_out_view.write(img);
                    }

                }
            }

        }

        mutex.post();

    }

public:

    ObjectRecognizerPort(ResourceFinder &rf) : BufferedPort<Image>()
    {
        // Binary file (.caffemodel) containing the network's weights
        string caffemodel_file = rf.check("caffemodel_file", Value("/path/to/model.caffemodel")).asString().c_str();
        cout << "Setting .caffemodel file to " << caffemodel_file << endl;

        // Text file (.prototxt) defining the network structure
        string prototxt_file = rf.check("prototxt_file", Value(" /path/to/deploy_imDataLayer.prototxt")).asString().c_str();
        cout << "Setting .prototxt file to " << prototxt_file << endl;

        // Name of blob to be extracted
        string blob_name = rf.check("blob_name", Value("prob")).asString().c_str();
        cout << "Setting blob_name to " << blob_name << endl;

        // Compute mode and eventually GPU ID to be used
        string compute_mode = rf.check("compute_mode", Value("GPU")).asString();
        int device_id = rf.check("device_id", Value(0)).asInt();

        int resize_width = rf.check("resize_width", Value(256)).asDouble();
        int resize_height = rf.check("resize_height", Value(256)).asDouble();

        caffe_wrapper = NULL;
        caffe_wrapper = new CaffeWrapper<float>(caffemodel_file, prototxt_file,
            resize_width, resize_height,
            blob_name,
            compute_mode, device_id);

            // labels
            string label_file = rf.check("label_file", Value("/path/to/labels.txt")).asString().c_str();;
            cout << "Setting labels.txt to " << label_file << endl;

            ifstream infile;

            string obj_name;
            vector<string> obj_names;
            int obj_idx;
            vector<int> obj_idxs;

            infile.open (label_file.c_str());
            infile >> obj_name;
            infile >> obj_idx;
            while (!infile.eof()) {
                std::cout << obj_name << " --> "<< obj_idx << std::endl;
                obj_names.push_back(obj_name);
                obj_idxs.push_back(obj_idx);
                infile >> obj_name;
                infile >> obj_idx;
            }
            infile.close();

            if (obj_names.size()!=obj_idxs.size())
            {
                std::cout << "label_file wrongly formatted!" << std::endl;
            }

            n_classes = obj_names.size();

            labels = new string[n_classes];
            for (int i=0; i<n_classes; i++)
            {
                labels[obj_idxs[i]] = obj_names[i];
            }

            // colors
            colors.push_back(cv::Scalar( 65, 47,213));
            colors.push_back(cv::Scalar(122, 79, 58));
            colors.push_back(cv::Scalar(154,208, 72));
            colors.push_back(cv::Scalar( 71,196,249));
            colors.push_back(cv::Scalar(224,176, 96));
            colors.push_back(cv::Scalar( 22,118,238));

            // parameters
            radius = 256;
            crop_mode = FIXED;
            img_crop_mat = cv::Mat(radius,radius,CV_8UC3);

            // this port
            BufferedPort<Image>::useCallback();

            // module name
            string name = rf.find("name").asString().c_str();

            // inout ports
            port_in_centroid.open(("/"+name+"/centroid:i").c_str());
            port_in_roi.open(("/"+name+"/roi:i").c_str());

            // output ports
            port_out_view.open(("/"+name+"/view:o").c_str());
            port_out_scores.open(("/"+name+"/scores:o").c_str());
            port_out_hist.open(("/"+name+"/hist:o").c_str());

        }

        bool set_radius(int _radius)
        {
            if (_radius>0)
            {
                mutex.wait();
                radius = _radius;
                mutex.post();
                return true;
            }
            else
            return false;
        }

        bool get_radius(int &_radius)
        {
            mutex.wait();
            _radius = radius;
            mutex.post();
        }

        bool set_crop_mode(int _crop_mode)
        {
            if (_crop_mode!=FIXED && _crop_mode!=CENTROID && _crop_mode!=ROI)
            return false;

            mutex.wait();
            crop_mode = _crop_mode;
            mutex.post();

            return true;
        }

        bool get_crop_mode(int &_crop_mode)
        {
            mutex.wait();
            _crop_mode = crop_mode;
            mutex.post();

            return true;
        }

        void interrupt()
        {
            mutex.wait();

            BufferedPort<Image>::interrupt();

            port_in_centroid.interrupt();
            port_in_roi.interrupt();

            port_out_view.interrupt();
            port_out_scores.interrupt();
            port_out_hist.interrupt();

            mutex.post();
        }

        void resume()
        {
            mutex.wait();

            BufferedPort<Image>::resume();

            port_in_centroid.resume();
            port_in_roi.resume();

            port_out_view.resume();
            port_out_scores.resume();
            port_out_hist.resume();

            mutex.post();
        }

        void close()
        {
            mutex.wait();

            BufferedPort<Image>::close();

            port_in_centroid.close();
            port_in_roi.close();

            port_out_view.close();
            port_out_scores.close();
            port_out_hist.close();

            delete[] labels;

            mutex.post();
        }

    };

    class ObjectRecognizerModule: public RFModule
    {
    protected:

        Semaphore              mutex;

        ObjectRecognizerPort   *imagePort;

        Port                   rpcPort;
        RpcServer              rpcPortHuman;

    public:

        ObjectRecognizerModule()
        {
            imagePort = NULL;
        }

        bool configure(ResourceFinder &rf)
        {

            Time::turboBoost();

            // module name
            string name = rf.find("name").asString().c_str();

            // input port
            imagePort = new ObjectRecognizerPort(rf);
            imagePort->open(("/"+name+"/img:i").c_str());

            // parameters
            int radius = rf.check("radius",Value(256)).asInt();
            int crop_mode = rf.check("crop_mode", Value(FIXED)).asInt();

            imagePort->set_radius(radius);
            imagePort->set_crop_mode(crop_mode);

            // rpc ports
            rpcPortHuman.open(("/"+name+"/human:io").c_str());

            rpcPort.open(("/"+name+"/rpc").c_str());
            attach(rpcPort);

            return true;
        }

        bool interruptModule()
        {
            if (imagePort!=NULL)
            imagePort->interrupt();

            rpcPort.interrupt();
            rpcPortHuman.interrupt();

            return true;
        }

        bool close()
        {
            if (imagePort!=NULL)
            {
                imagePort->close();
                delete imagePort;
            }

            rpcPort.close();
            rpcPortHuman.close();

            return true;
        }

        bool respond(const Bottle &command, Bottle &reply)
        {
            return RFModule::respond(command,reply);
        }

        double getPeriod()    { return 1.0;  }

        bool updateModule()
        {
            bool ok = false;

            Bottle command,reply;
            rpcPortHuman.read(command,true);

            if (command.size()>0)
            {

                mutex.wait();

                switch(command.get(0).asVocab())
                {

                    case CMD_HELP:
                    {
                        reply.addVocab(Vocab::encode("many"));
                        reply.addString(" ");
                        reply.addString("set crop_mode fixed      : sets the square crop at the center");
                        reply.addString("set crop_mode centroid   : sets the square crop on the object");
                        reply.addString("set crop_mode roi        : sets the rectangular crop around the object");
                        reply.addString(" ");
                        reply.addString("set radius <value>       [ int>0  ]: sets the square radius if 'centroid' or 'fixed' mode is on");
                        reply.addString(" ");
                        reply.addString("get radius               : provides the radius of the square ROI, if 'radius' mode is on");
                        reply.addString("get crop_mode            : tells if 'fixed', 'centroid', 'roi' mode is on");

                        ok = true;

                    } break ;

                    case CMD_SET:
                    {
                        if (command.size()>2)
                        {
                            string property = command.get(1).asString().c_str();

                            if (property == "radius")
                            {
                                int r = command.get(2).asInt();
                                ok = imagePort->set_radius(r);
                            }
                            else if (property == "crop_mode")
                            {
                                int cm = command.get(2).asVocab();
                                ok = imagePort->set_crop_mode(cm);
                            }
                            else
                            {
                                ok = false;
                                reply.addString("Unknown property.");
                                break;
                            }

                        } else
                        {
                            ok = false;
                            reply.addString("Syntax must be: set <prop> <value>");
                            break;
                        }

                        if (ok)
                        reply.addVocab(ACK);
                        else
                        reply.addString("Cannot set property: check value.");

                        break;

                    }

                    case CMD_GET:
                    {
                        if (command.size()>1)
                        {
                            string property = command.get(1).asString().c_str();

                            if (property=="radius")
                            {
                                int r;
                                ok = imagePort->get_radius(r);
                                reply.addInt(r);
                                break;
                            }
                            if (property=="crop_mode")
                            {
                                int cm;
                                ok = imagePort->get_crop_mode(cm);
                                reply.addVocab(cm);
                                break;
                            }
                            else
                            {
                                ok = false;
                                reply.addString("Unknown property.");
                                break;
                            }
                        }
                        else
                        {
                            ok = false;
                            reply.addString("Syntax must be: get <property>");
                            break;
                        }

                    }

                    default:
                    reply.addString("Unknown command!");

                }

                mutex.post();

                rpcPortHuman.reply(reply);

            }

            return true;
        }

    };


    int main(int argc, char *argv[])
    {
        Network yarp;

        if (!yarp.checkNetwork())
        return 1;

        ResourceFinder rf;

        rf.setVerbose(true);

        rf.setDefaultContext("objectRecognizer");
        rf.setDefaultConfigFile("objectRecognizer.ini");

        rf.configure(argc,argv);

        rf.setDefault("name","objectRecognizer");

        ObjectRecognizerModule mod;

        return mod.runModule(rf);
    }
