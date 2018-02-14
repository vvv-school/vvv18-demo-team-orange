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

#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class GazeCtrl: public RFModule 
{
    Port rpcPort;

protected:
    PolyDriver drvGaze;
    IGazeControl *igaze;

    bool simulation;
    
    yarp::os::BufferedPort<yarp::sig::Vector> portKinematicsLookAt;

    /***************************************************/
    void fixGaze(const Vector &x)
    {
        if (!simulation)
            igaze->blockEyes(5.0);

        igaze->lookAtFixationPointSync(x);
        igaze->waitMotionDone();
    }

public:
    /***************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        rpcPort.open("/service");
        attach(rpcPort);

        string robot=rf.check("robot",Value("icubSim")).asString();
        simulation=(robot=="icubSim");

        // GAZE CONFIGURATION
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/tracker/gaze");

        if (!drvGaze.open(optGaze))
        {
            yError()<<"Unable to open the Gaze Controller";
            return false;
        }

        portKinematicsLookAt.open("/orange/kinematics_look_at:i");

        // open the view
        drvGaze.view(igaze);

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        portKinematicsLookAt.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        portKinematicsLookAt.close();
        return true;
    }

    /***************************************************/
    bool updateModule()
    {   
        yarp::sig::Vector *look_at = portKinematicsLookAt.read();
        //Vector look_at(3);
        //look_at[0] = 0.15;
        //look_at[1] = 0;
        //look_at[2] = 0;
        fixGaze(*look_at);
        std::cout << "Looking down" << std::endl;
        return true;
    }
};


/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    GazeCtrl mod;
    return mod.runModule(rf);
}

