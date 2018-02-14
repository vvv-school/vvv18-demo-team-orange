//
// A tutorial on how to wrap Caffe in YARP and recognize objects in images.
//
// Author: Giulia Pasquale - <giulia.pasquale@iit.it>

#ifndef CAFFEWRAPPER_H_
#define CAFFEWRAPPER_H_

#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

// Boost
#include "boost/algorithm/string.hpp"
#include "boost/make_shared.hpp"

// Caffe
#include "caffe-version.h"

#if (CAFFE_MAJOR >= 1)
// new caffe headers
#include "caffe/caffe.hpp"
#include "caffe/layers/memory_data_layer.hpp"
#else
// old caffe headers
#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/net.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/io.hpp"
#include "caffe/vision_layers.hpp"
#endif

using namespace std;
using namespace caffe;

template<class Dtype>
class CaffeWrapper {

    string caffemodel_file;
    string prototxt_file;

    caffe::shared_ptr<Net<Dtype> > deployment_net;

    int mean_width;
    int mean_height;
    int mean_channels;

    string blob_name;

    bool gpu_mode;
    int device_id;

public:

    CaffeWrapper(string _caffemodel_file,
        string _prototxt_file, int _resize_width, int _resize_height, string blob_name,
        string _compute_mode, int _device_id);

        bool forward(cv::Mat &image, vector<Dtype> &features);
    };

    template <class Dtype>
    CaffeWrapper<Dtype>::CaffeWrapper(string _caffemodel_file,
        string _prototxt_file, int _resizeWidth, int _resizeHeight,
        string _blob_name,
        string _compute_mode,
        int _device_id) {

            // Setup the GPU or the CPU mode for Caffe
            if (strcmp(_compute_mode.c_str(), "GPU") == 0 || strcmp(_compute_mode.c_str(), "gpu") == 0) {

                std::cout << "CaffeWrapper::CaffeWrapper(): using GPU" << std::endl;

                gpu_mode = true;
                device_id = _device_id;

                Caffe::CheckDevice(device_id);

                std::cout << "CaffeWrapper::CaffeWrapper(): using device_id = " << device_id << std::endl;

                Caffe::set_mode(Caffe::GPU);
                Caffe::SetDevice(device_id);

                // Optional: to check that the GPU is working properly...
                Caffe::DeviceQuery();

            } else
            {
                std::cout << "CaffeWrapper::CaffeWrapper(): using CPU" << std::endl;

                gpu_mode = false;
                device_id = -1;

                Caffe::set_mode(Caffe::CPU);
            }

            // Assign specified .caffemodel and .prototxt files
            caffemodel_file = _caffemodel_file;
            prototxt_file = _prototxt_file;

            // Network creation using the specified .prototxt
            deployment_net = boost::make_shared<Net<Dtype> > (prototxt_file, caffe::TEST);

            // Network initialization using the specified .caffemodel
            deployment_net->CopyTrainedLayersFrom(caffemodel_file);

            // Mean image initialization

            mean_width = 0;
            mean_height = 0;
            mean_channels = 0;

            caffe::shared_ptr<MemoryDataLayer<Dtype> > memory_data_layer = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<Dtype> >(deployment_net->layers()[0]);

            TransformationParameter tp = memory_data_layer->layer_param().transform_param();

            if (tp.has_mean_file())
            {
                const string& mean_file = tp.mean_file();
                std::cout << "CaffeWrapper::CaffeWrapper(): loading mean file from " << mean_file << std::endl;

                BlobProto blob_proto;
                ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

                Blob<Dtype> data_mean;
                data_mean.FromProto(blob_proto);

                mean_channels = data_mean.channels();
                mean_width = data_mean.width();
                mean_height = data_mean.height();

            } else if (tp.mean_value_size()>0)
            {

                const int b = tp.mean_value(0);
                const int g = tp.mean_value(1);
                const int r = tp.mean_value(2);

                mean_channels = tp.mean_value_size();
                mean_width = _resizeWidth;
                mean_height = _resizeHeight;

                std::cout << "CaffeWrapper::CaffeWrapper(): B " << b << "   G " << g << "   R " << r << std::endl;
                std::cout << "CaffeWrapper::CaffeWrapper(): resizing anysotropically to " << " W: " << mean_width << " H: " << mean_height << std::endl;
            }
            else
            {
                std::cout << "CaffeWrapper::CaffeWrapper(): Error: neither mean file nor mean value in prototxt!" << std::endl;
            }

            blob_name = _blob_name;
            // Check that requested blobs exist
            if (!deployment_net->has_blob(blob_name))
            {
                std::cout << "CaffeWrapper::CaffeWrapper(): unknown feature blob name " << blob_name << " in the network " << prototxt_file << std::endl;
            }

        }

        template<class Dtype>
        bool CaffeWrapper<Dtype>::forward(cv::Mat &image, vector<Dtype> &features)
        {
            // Check input image
            if (image.empty())
            {
                std::cout << "CaffeWrapper::forward(): empty imMat!" << std::endl;
                return false;
            }

            // Set the GPU/CPU mode for Caffe (here in order to be thread-safe)
            if (gpu_mode)
            {
                Caffe::set_mode(Caffe::GPU);
                Caffe::SetDevice(device_id);
            }
            else
            {
                Caffe::set_mode(Caffe::CPU);
            }

            // Initialize labels to zero
            int label = 0;

            // Get pointer to data layer to set the input
            caffe::shared_ptr<MemoryDataLayer<Dtype> > memory_data_layer = boost::dynamic_pointer_cast<caffe::MemoryDataLayer<Dtype> >(deployment_net->layers()[0]);

            // Set batch size to 1
            if (memory_data_layer->batch_size()!=1)
            {
                memory_data_layer->set_batch_size(1);
                std::cout << "CaffeWrapper::forward(): BATCH SIZE = " << memory_data_layer->batch_size() << std::endl;
            }

            // Image preprocessing
            // The image passed to AddMatVector must be same size as the mean image
            // If not, it is resized:
            // if it is downsampled, an anti-aliasing Gaussian Filter is applied
            if (image.rows != mean_height || image.cols != mean_height)
            {
                if (image.rows > mean_height || image.cols > mean_height)
                {
                    cv::resize(image, image, cv::Size(mean_height, mean_width), 0, 0, CV_INTER_LANCZOS4);
                }
                else
                {
                    cv::resize(image, image, cv::Size(mean_height, mean_width), 0, 0, CV_INTER_LINEAR);
                }
            }

            memory_data_layer->AddMatVector(vector<cv::Mat>(1, image),vector<int>(1,label));

            // Run network and retrieve features!

            // depending on your net's architecture, the blobs will hold accuracy and/or labels, etc
            std::vector<Blob<Dtype>*> results = deployment_net->Forward();

            const caffe::shared_ptr<Blob<Dtype> > feature_blob = deployment_net->blob_by_name(blob_name);

            int batch_size = feature_blob->num(); // should be 1
            if (batch_size!=1)
            {
                std::cout << "CaffeWrapper::forward(): Error! Retrieved more than one feature, exiting..." << std::endl;
                return -1;
            }

            int feat_dim = feature_blob->count(); // should be equal to: count/num=channels*width*height
            if (feat_dim!=feature_blob->channels())
            {
                std::cout << "CaffeWrapper::forward(): Attention! The feature is not 1D: unrolling according to Caffe's order (i.e. channel, height, width)" << std::endl;
            }

            features.insert(features.end(), feature_blob->mutable_cpu_data() + feature_blob->offset(0), feature_blob->mutable_cpu_data() + feature_blob->offset(0) + feat_dim);

            return true;
            
        }

        #endif /* CAFFEWRAPPER_H_ */
