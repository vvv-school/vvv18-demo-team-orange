#include "kinematics.h"

bool ClientMod::configPorts()
{
    // open all ports and check that everything is fine
    // output port: /client/output
    // input port: /client/input
    if (!inPort.open("/client/input")) {
        yError() << "cannot open the input port";
        return -1;
    }

    if (!outPort.open("/client/output")) {
        yError() << "cannot open the output port";
        return -1;
    }

    return true;
}

/****************************************************/
bool ClientMod::configure(ResourceFinder &rf)
{
    // get "angle" input parameter
    // configure the ports
    bool conf = configPorts();

    // Check if angle is passed by command line argument
    if (rf.check("angle"))
    {
        angle  = rf.find("angle").asDouble();
    }

    // Check if period is passed by command line argument
    if (rf.check("period"))
    {
        period = rf.find("period").asDouble();
    }
    return conf;
}

/****************************************************/
double ClientMod::getPeriod()
{
    return period;
}

/****************************************************/
bool ClientMod::close()
{
    // close ports
    inPort.close();
    outPort.close();
    return true;
}

/****************************************************/
bool ClientMod::interrupt()
{
    // interrupt ports
    inPort.interrupt();
    outPort.interrupt();
    return true;
}

/****************************************************/
bool ClientMod::updateModule()
{
    if (!triggered)
    {
        // read from the input port the signal from the
        // trigger for starting to send data to the server
        Bottle *input = inPort.read();
        if(input == NULL) { 
            yError() << "Error reading from input port";
        }
        else {
            triggered = true;
            yInfo() << "triggered";
        }
    }

    // once triggered prepare the bottle containing the
    // angle and send it to the server through the
    // output port
    if(triggered) {
        // prepare the output data 
        Bottle &output = outPort.prepare();
        output.addDouble(angle);
        output.addDouble(period);
        outPort.write();
        yInfo() << "Writing client to server";
    }
    return true;
}