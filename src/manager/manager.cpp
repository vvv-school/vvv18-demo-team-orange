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
        rpcKinematicsHighFive.open("/orange/kinematics_high_five:o");
        rpcDynamicsFeedback.open("/orange/dynamics_feedback:o");

        return true;
    }

    /*
    * Main function, called periodically every getPeriod() seconds.
    */
    bool updateModule()
    {
        // wait for user input
        yInfo() << "input: wating...";


        yInfo() << "input: got it!";


        // initial face expression
        yInfo() << "face expression: initial \n";
        yarp::os::Bottle face_expression_ini;
        face_expression_ini.addString("set");
        face_expression_ini.addString("all");
        face_expression_ini.addString("sur");
        portKinematicsFaceExpression.write(face_expression_ini);


        Time::delay(2.0);


        // look down to table
        yInfo() << "look_down: request";
        yarp::os::Bottle request_ld, response_ld;
        request_ld.addString("look_at");
        request_ld.addDouble(40);
        rpcKinematicsHighFive.write(request_ld, response_ld);
        yInfo() << "look_down: finished \n";


        Time::delay(2.0);


        // read input from vision



        // send bounding boxes (or cropped images) to classifier



        // check bounding boxes content with input


        // point to
        yInfo() << "point to: request";
        yarp::os::Bottle request_pt, response_pt;
        request_ld.addString("point_to");
        request_ld.addDouble(-0.3);
        request_ld.addDouble(0.0);
        request_ld.addDouble(0.0);
        rpcKinematicsHighFive.write(request_pt, response_pt);
        yInfo() << "point to: finished \n";


        Time::delay(2.0);


        // high-five
        yInfo() << "high-five: request";
        yarp::os::Bottle request_hf, response_hf;
        request_hf.addString("high_five");
        rpcKinematicsHighFive.write(request_hf, response_hf);
        yInfo() << "high-five: finished \n";


        Time::delay(2.0);


        // ask for feedback
        yInfo() << "feedback: request";
        yarp::os::Bottle request_feed, response_feed;
        request_feed.addString("Give me feedback");
        rpcDynamicsFeedback.write(request_feed, response_feed);
        yInfo() << "feedback: finished \n";


        // react accordingly to feedback
        yarp::os::Bottle face_expression;
        bool feedback = response_feed.get(0).asBool();
        feedback = false;
        if (feedback) {
            // be happy
            yInfo() << "I am happy :)! \n";
            face_expression.addString("set");
            face_expression.addString("all");
            face_expression.addString("hap");
        }
        else {
            // be sad
            yInfo() << "I am sad :(! \n";
            face_expression.addString("set");
            face_expression.addString("all");
            face_expression.addString("sad");
        }
        portKinematicsFaceExpression.write(face_expression);


        // end of cicle, wait before restarting...
        yInfo() << "End of cicle, wait before restarting... \n\n\n";
        Time::delay(5.0);


        // home position
        yInfo() << "home: request \n";
        yarp::os::Bottle request_hp, response_hp;
        request_hp.addString("home");
        //rpcKinematicsHighFive.write(request_hp, response_hp);
        yInfo() << "home: finished \n";

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