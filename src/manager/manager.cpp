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

    yarp::os::Port portKinematicsFaceExpression;
    
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

        portKinematicsFaceExpression.open("/orange/kinematics_face_expression:o");

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

        yarp::os::Bottle face_expression_ini;
        face_expression_ini.addString("set");
        face_expression_ini.addString("all");
        face_expression_ini.addString("sur");
        portKinematicsFaceExpression.write(face_expression_ini);

        Time::delay(2.0);



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
        yarp::os::Bottle face_expression;
        bool feedback = response_feed.get(0).asBool();
        feedback = false;
        if (feedback) {
            // be happy
            yInfo() << "I am happy :)!";
            face_expression.addString("set");
            face_expression.addString("all");
            face_expression.addString("hap");
        }
        else {
            // be sad
            yInfo() << "I am sad :(!";
            face_expression.addString("set");
            face_expression.addString("all");
            face_expression.addString("sad");
        }
        portKinematicsFaceExpression.write(face_expression);

        // end of cicle, wait before restarting...
        yInfo() << "End of cicle, wait before restarting...";
        Time::delay(5.0);

        // home position
        yInfo() << "home: request";
        yarp::os::Bottle request_hp, response_hp;
        request_hp.addString("home");
        //rpcKinematicsHighFive.write(request_hp, response_hp);
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

    yarp.connect("/orange/kinematics_face_expression:o", "/emotion/in");

    yarp.connect("/orange/kinematics_high_five:o", "/orange/kinematics_high_five:i");
    yarp.connect("/orange/dynamics_feedback:o", "/orange/dynamics_feedback:i");

    yarp.connect("/face/eyelids", "/icubSim/face/eyelids");
    yarp.connect("/face/image/out", "/icubSim/texture/face");
    yarp.connect("/emotion/out", "/icubSim/face/raw/in");


    yInfo()<<"Start module...";
    manager.runModule();

    yInfo()<<"Main returning...";
    return 0;
}