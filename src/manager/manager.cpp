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

#include <iostream>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>



using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class Manager:public RFModule
{
    double period;
    
    yarp::os::BufferedPort<yarp::sig::Vector> portKinematicsLookAt;
    yarp::os::BufferedPort<yarp::sig::Vector> portKinematicsPointTo;

    yarp::os::RpcClient rpcKinematicsHighFive;
    yarp::os::RpcClient rpcDynamicsFeedback;

public:

    /*
    * Module periodicity (seconds)
    */
    double getPeriod()
    {
        return period;
    }

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf)
    {
    	// resource finder to parse parameter --period
        period = 0.1;
        if (rf.check("period"))
            period = rf.find("period").asDouble();

        //rpcManager.open("orange/manager:rpc");

        portKinematicsLookAt.open("/orange/kinematics_look_at:o");
        portKinematicsPointTo.open("/orange/kinematics_point_to:o");

        rpcKinematicsHighFive.open("/orange/kinematics_high_five:o");
        rpcDynamicsFeedback.open("/orange/dynamics_feedback:o");

        return true;
    }

    /*
    * Main function, called periodically every getPeriod() seconds.
    */
    bool updateModule()
    {
        std::cout <<  "Manager: running happily..." << std::endl;
        yInfo() << "Manager: running happily...";


        // look down to table
        /*
        yarp::sig::Vector &lookAt = portKinematicsLookAt.prepare();
        lookAt = yarp::math::zeros(3);
        lookAt[0] = -0.15;
        lookAt[1] = 0;
        lookAt[2] = 0;
        portKinematicsLookAt.writeStrict();

        yInfo() << "look_down: request";
        yarp::os::Bottle request_ld, response_ld;
        request_ld.addString("look_at");
        request_ld.addDouble(-0.15);
        request_ld.addDouble(0.0);
        request_ld.addDouble(0.0);
        rpcKinematicsHighFive.write(request_ld, response_ld);
        yInfo() << "look_down: finished";
        */



        // read input from vision



        // send bounding boxes (or cropped images) to classifier



        // check bounding boxes content with input


        // point (kinematics)
        /*
        yarp::sig::Vector &pointTo = portKinematicsPointTo.prepare();
        pointTo = yarp::math::zeros(3);
        pointTo[0] = -0.3;
        pointTo[1] = 0.2;
        pointTo[2] = 0;
        portKinematicsPointTo.writeStrict();
        */


        // high-five
        yInfo() << "high-five: request";
        yarp::os::Bottle request_hf, response_hf;
        request_hf.addString("high_five");
        rpcKinematicsHighFive.write(request_hf, response_hf);
        yInfo() << "high-five: finished";


        // ask for feedback
        yInfo() << "feedback: request";
        yarp::os::Bottle request_feed, response_feed;
        request_feed.addString("Give me feedback");
        rpcDynamicsFeedback.write(request_feed, response_feed);
        yInfo() << "feedback: finished";


        // react accordingly to feedback
        bool feedback = response_feed.get(0).asBool();
        if (feedback) {
            yInfo() << "I am happy :)!";
            // be happy
        }
        else {
            // be sad
            yInfo() << "I am sad :(!";
        }


        // home position
        yInfo() << "home: request";
        yarp::os::Bottle request_hf, response_hf;
        request_hf.addString("home");
        rpcKinematicsHighFive.write(request_hf, response_hf);
        yInfo() << "home: finished";

        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        yInfo() << "Interrupting your module, for port cleanup";
        portKinematicsLookAt.interrupt();
        portKinematicsPointTo.interrupt();
        return true;
    }

    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        yInfo() << "Calling close function";
        portKinematicsLookAt.close();
        portKinematicsPointTo.close();
        return true;
    }
};

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    Manager manager;
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    // rf.setVerbose(true);

    yInfo()<<"Configure module...";
    manager.configure(rf);


    yarp.connect("/orange/kinematics_high_five:o", "/orange/kinematics_high_five:i");
    yarp.connect("/orange/dynamics_feedback:o", "/orange/dynamics_feedback:i");

    yarp.connect("")


    yInfo()<<"Start module...";
    manager.runModule();

    yInfo()<<"Main returning...";
    return 0;
}