// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>

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
class IKinematics: public RFModule 
{
protected:
    PolyDriver drvArm;
    ICartesianControl *iarm;

    RpcServer rpcPort;

    Mutex mutex;

    bool simulation;


    Vector computeHandOrientation()
    {
        // FILL IN THE CODE
        Matrix R(3, 3);
        R(0, 0) = 0; R(0, 1) = 0; R(0, 2) = -1;
        R(1, 0) = 0; R(1, 1) = 1; R(1, 2) = 0;
        R(2, 0) = 1; R(2, 1) = 0; R(2, 2) = 0;
        return dcm2axis(R);
    }

    /***************************************************/
    void highFive(const Vector &x, const Vector &o)
    {
        bool ok = iarm->goToPoseSync(x, o);
        iarm->waitMotionDone();
        yInfo() << "Inside function";
        //yInfo() << ok?"true":"false";
    }

    /***************************************************/
    void home()
    {
        // FILL IN THE CODE
        //iarm->goToPoseSync(init_x, init_o);
        //iarm->waitMotionDone();
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();
        simulation=(robot=="icubSim");

        /*
        * ARM CONFIGURATION
        */
        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/"+robot+"/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        drvArm.view(iarm);
        iarm->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        iarm->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;

        // send the request for dofs reconfiguration
        iarm->setDOF(newDof,curDof);

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        /*
        * PORTS CONFIGURATION
        */
        rpcPort.open("/service");
        attach(rpcPort);

        return true;
    }

    /***************************************************/
    void limitTorsoPitch()
    {
        int axis = 0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we dson't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        iarm->getLimits(axis, &min, &max);
        iarm->setLimits(axis, min, 30);
    }

    /***************************************************/
    bool interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- high_five");
        }
        else if (cmd=="high_five")
        {
            //-0.0247262931419455 0.330151552412954 0.373188508814116 
            //0.016828116386148 -0.941893631916566 0.335489494103637 1.34803603197536

            Vector pose(3);
            pose[0] = -0.025;
            pose[1] = 0.33;
            pose[2] = 0.37;

            //Vector orie(4);
            //orie[0] = 0.017;
            //orie[1] = -0.94;
            //orie[2] = 0.335;
            //orie[3] = 1.35;
            // make sure hand is perpendicularly oriented
            Vector orie = computeHandOrientation();

            highFive(pose, orie);

            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! Ready to rock!");
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {
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
    rf.configure(argc,argv);

    IKinematics mod;
    return mod.runModule(rf);
}
