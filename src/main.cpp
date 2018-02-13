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

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class CtrlModule: public RFModule 
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    bool simulation;
    ObjectRetriever object;

    Vector init_x, init_o;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        // FILL IN THE CODE
        Vector pose_3D(3);
        igaze->triangulate3DPoint(cogL, cogR, pose_3D);
        return pose_3D;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        // FILL IN THE CODE
        igaze->lookAtFixationPointSync(x);
        igaze->waitMotionDone();
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        // FILL IN THE CODE
        Matrix R(3, 3);
        R(0, 0) = -1; R(0, 1) = 0; R(0, 2) = 0;
        R(1, 0) = 0; R(1, 1) = 0; R(1, 2) = -1;
        R(2, 0) = 0; R(2, 1) = -1; R(2, 2) = 0;
        return dcm2axis(R);
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        // FILL IN THE CODE
        Vector pose_3D(3);
        pose_3D[0] = x[0];
        pose_3D[1] = x[1] + 0.2;
        pose_3D[2] = x[2]; 

        iarm->goToPoseSync(pose_3D, o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void roll(const Vector &x, const Vector &o)
    {
        // FILL IN THE CODE
        Vector pose_3D(3);
        pose_3D[0] = x[0];
        pose_3D[1] = x[1] - 0.2;
        pose_3D[2] = x[2]; 

        iarm->goToPoseSync(pose_3D, o);
        iarm->waitMotionDone();

    }

    /***************************************************/
    void look_down()
    {
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
        if (!simulation)
            igaze->blockEyes(5.0);

        // FILL IN THE CODE
        Vector pose_3D(3);
        pose_3D[0] = -0.150;
        pose_3D[1] = 0;
        pose_3D[2] = 0;
        igaze->lookAtFixationPointSync(pose_3D);
        igaze->waitMotionDone();
    }

    /***************************************************/
    bool make_it_roll(const Vector &cogL, const Vector &cogR)
    {
        Vector x;
        if (simulation)
        {
            yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";
            x=retrieveTarget3D(cogL,cogR);
        }
        else if (!object.getLocation(x))
            return false;

        yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o);
        yInfo()<<"approached";

        roll(x,o);
        yInfo()<<"roll!";

        return true;
    }

    /***************************************************/
    void home()
    {
        // FILL IN THE CODE
        iarm->goToPoseSync(init_x, init_o);
        iarm->waitMotionDone();
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

        iarm->getPose(init_x, init_o);

        /*
        * GAZE CONFIGURATION
        */
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/tracker/gaze");

        if (!drvGaze.open(optGaze))
        {
            yError()<<"Unable to open the Gaze Controller";
            return false;
        }

        // open the view
        drvGaze.view(igaze);


        /*
        * PORTS CONFIGURATION
        */
        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

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
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
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
            reply.addString("- look_down");
            reply.addString("- make_it_roll");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");

            Time::delay(1.0);
        }
        else if (cmd=="make_it_roll")
        {
            // FILL IN THE CODE
            bool go=okL && okR;   // you need to properly handle this flag

            bool rolled=false;
            if (go || !simulation)
                rolled=make_it_roll(cogL,cogR);
            // we assume the robot is not moving now

            if (rolled)
            {
                reply.addString("ack");
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd=="home")
        {
            home();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("I've got the hard work done! Gone home.");
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
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return false;

        // compute the center-of-mass of pixels of our color
        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.prepare()=*imgL;
        imgRPortOut.prepare()=*imgR;

        imgLPortOut.write();
        imgRPortOut.write();

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

    CtrlModule mod;
    return mod.runModule(rf);
}
