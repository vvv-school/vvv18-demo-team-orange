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

        portKinematicsLookAt.open("/orange/kinematics_look_at:o");
        portKinematicsPointTo.open("/orange/kinematics_point_to:o");

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
        std::cout << "Trying to look down 2" << std::endl;
        yarp::sig::Vector &lookAt = portKinematicsLookAt.prepare();
        lookAt = yarp::math::zeros(3);
        lookAt[0] = -0.15;
        lookAt[1] = 0;
        lookAt[2] = 0;
        portKinematicsLookAt.writeStrict();
        std::cout << "Trying to look down 2" << std::endl;


        // read input from vision



        // send bounding boxes (or cropped images) to classifier



        // check bounding boxes content with input


        // point (kinematics)
        yarp::sig::Vector &pointTo = portKinematicsPointTo.prepare();
        pointTo = yarp::math::zeros(3);
        pointTo[0] = -0.3;
        pointTo[1] = 0.2;
        pointTo[2] = 0;
        portKinematicsPointTo.writeStrict();


        // high-five or low-five



        // wait for feedback



        // react 



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
    yInfo()<<"Start module...";
    manager.runModule();

    yInfo()<<"Main returning...";
    return 0;
}