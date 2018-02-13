#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl2.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


class ClientMod : public yarp::os::RFModule
{
private:

    // define one output port to comunicate to the server and
    // one input port to receive the trigger to start moving the
    // arm
    // hint: think about which port you need, buffered? simple? both?
    double angle, period;
    bool triggered;

    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;


public:

    // set the correct angle to send to the server and the period of the thread
    ClientMod();// : angle(31.0) : period(2.0) : triggered(false)

    /****************************************************/
    bool configPorts();

    /****************************************************/
    bool configure(ResourceFinder &rf);

    /****************************************************/
    double getPeriod();

    /****************************************************/
    bool close();

    /****************************************************/
    bool interrupt();

    /****************************************************/
    bool updateModule();
};