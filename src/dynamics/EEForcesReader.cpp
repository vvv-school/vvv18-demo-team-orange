#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/impl/Logger.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <string>

//#include <yarp/os/all.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, const char **argv) {

    // initialize yarp network
    Network yarp;

    RpcClient resetForces;
    resetForces.open("/forcesReader");
    yarp.connect("/forcesReader","/wholeBodyDynamics/rpc:i");

    yInfo() << "Resetting the sensors to 0...\n";
    Bottle cmd, response;
    cmd.addInt(0);
    resetForces.write(cmd, response);
    Time::delay(0.5);
    yInfo() << "Sensors resetted!\n";

    BufferedPort<Vector> reader;
    if (!reader.open("/EEForcesReader")) {
        yError() << "cannot open the input port";
        return -1;
    }
    yarp.connect("/EEForcesReader","/wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o");


    while (true) {
        Vector *input;
        if ((input = reader.read()) == false) {
            Time::delay(0.1);
        }else{
            double Xforce = (*input)[0];
            yInfo() << "Force along the x-axis: " << Xforce;
            if (Xforce > 1){
                yError() << "Positive";
            }else if (Xforce < -1){
                yError() << "Negative";
            }
        }


    }

    return 0;
}


/*
            while (true) {
                if (port.getOutputCount()==0) {
                  printf("Trying to connect to %s\n", server_name);
                  yarp.connect(client_name,server_name);
                } else {
                  Bottle cmd;
                  cmd.addString("COUNT");
                  cmd.addInt(ct);
                  ct++;
                  printf("Sending message... %s\n", cmd.toString().c_str());
                  Bottle response;
                  port.write(cmd,response);
                  printf("Got response: %s\n", response.toString().c_str());
                }
                Time::delay(1);
            }
*/
