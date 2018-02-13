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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   inPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   cropOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  targetPort;

    yarp::os::RpcClient rpc;

public:
    /********************************************************/

    Processing(const std::string &moduleName);

    /********************************************************/
    ~Processing();

    /********************************************************/
    bool open();

    /********************************************************/
    void close();

    /********************************************************/
    void interrupt();

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage );
};

/********************************************************/
class Module : public yarp::os::RFModule
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source);
public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /**********************************************************/
    bool close();

    /**********************************************************/
    bool quit();

    /********************************************************/
    double getPeriod();

    /********************************************************/
    bool updateModule();
};