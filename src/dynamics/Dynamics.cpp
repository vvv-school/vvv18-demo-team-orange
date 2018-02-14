#include <iostream>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/impl/Logger.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <string>

#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

int queryContact(Vector F, double threshold, double ratio){

    double Fx = F(0);
    double normF = yarp::math::norm(F);

    if ((normF > threshold) && (Fx/normF > ratio)){
        return 1;
    }else if ((normF > threshold) && (Fx/normF < -ratio)){
        return 2;
    }else{
        return 0;
    }
}

class MyModule:public RFModule{

    RpcServer handlerPort; //a port to handle messages
    RpcClient resetForces; //to communicate with the wholeBodyDynamics in order to reset the sensors

    BufferedPort<Vector> reader;

    double period = 0.5;
    double threshold = 4.0;
    double ratio = 0.8;

public:

    bool respond(const Bottle& command, Bottle& reply){
        yInfo() << "Received ok to start. I'm reading the forces applied to the EE...";
        while (true) {
            Vector *input = reader.read(false);
            if (input != NULL){
                int result = queryContact((*input), threshold, ratio);
                if (result == 0){
                    yError() << "No contact...just noise";
                }else if (result == 1){
                    yError() << "Positive";
                }else{
                    yError() << "Positive";
                }
            }else{
                Time::yield();
            }

        }
    }

    bool configure(yarp::os::ResourceFinder &rf){

        Network yarp;

        handlerPort.open("/orange/dynamics_feedback:i");
        attach(handlerPort);

        resetForces.open("/forcesReader");
        yarp.connect("/forcesReader","/wholeBodyDynamics/rpc:i");

        yInfo() << "Resetting the sensors to 0...\n";
        Bottle cmd, response;
        cmd.addInt(0);
        resetForces.write(cmd, response);
        Time::delay(0.5);
        yInfo() << "Sensors resetted!\n";

        if (!reader.open("/EEForcesReader")) {
            yError() << "cannot open the input port";
            return -1;
        }
        yarp.connect("/wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o", "/EEForcesReader");

        return true;
    }

    bool updateModule(){
        yInfo( )<< "Running fine...";
        return true;
    }

    double getPeriod(){
        return period; //module periodicity (seconds)
    }

    bool interruptModule(){
        yInfo() << "Interrupting module for port cleanup";
        return true;
    }

    bool close(){
        yInfo() << "Calling close function";
        resetForces.close();
        handlerPort.close();
        return true;
    }
    
};


int main(int argc, char * argv[]){
    Network yarp;

    MyModule module;
    ResourceFinder rf;
    rf.configure(argc, argv);

    yInfo()<<"Configure module...";
    module.configure(rf);
    yInfo()<<"Start module...";
    module.runModule();

    yInfo()<<"Main returning...";
    return 0;
}


