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

    yarp::os::Port portSpeechInput;
    yarp::os::Port portKinematicsFaceExpression;

    yarp::os::BufferedPort<yarp::os::Bottle> portVision;

    yarp::os::BufferedPort<yarp::os::Bottle> portClassifierROI;
    yarp::os::BufferedPort<yarp::os::Bottle> portClassifierLabel;

    yarp::os::BufferedPort<yarp::sig::Vector> portKinematicsLookAt;
    yarp::os::BufferedPort<yarp::sig::Vector> portKinematicsPointTo;

    yarp::os::RpcClient rpcKinematicsHighFive;
    yarp::os::RpcClient rpcDynamicsFeedback;
    
    //yarp::os::BufferedPort<yarp::os::Bottle> exitStatusBottle;

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

        portSpeechInput.open("/orange/speech:i");
        portVision.open("/orange/vision/controller:i");
        portClassifierROI.open("/orange/portClassifierROI:o");
        portClassifierLabel.open("/orange/portClassifierLabel:i");

        portKinematicsFaceExpression.open("/orange/kinematics_face_expression:o");
        rpcKinematicsHighFive.open("/orange/kinematics_high_five:o");
        rpcDynamicsFeedback.open("/orange/dynamics_feedback:o");
        
        //exitStatusPort.open("orange/manager:o");

        return true;
    }

    /*
    * Main function, called periodically every getPeriod() seconds.
    */

    bool updateModule()
    {
        // USER INPUT 
        std::string desired_object = "";
        yInfo() << "input: wating...";

        yarp::os::Bottle user_input;
        std::string user_cmd; // = user_input->get(0).asString();
        if (!portSpeechInput.read(user_input)) {
            return true;
        }
        else {
            user_cmd = user_input.get(0).asString();
        }

        yInfo() << "input: " << user_cmd;

        if (user_cmd == "home") {
            // HOME POSITION
            yInfo() << "home: request \n";
            yarp::os::Bottle request_hp, response_hp;
            request_hp.addString("home");
            rpcKinematicsHighFive.write(request_hp, response_hp);
            yInfo() << "home: finished \n";
        }
        else if (user_cmd == "quit") {

        }
        else if (user_cmd == "where") {
            desired_object = user_input.get(1).asString();
        }
        else {
            yWarning() << "Command not valid!";
            return true;
        }
        yInfo() << "input: got it!";

        /*
        // INITIAL FACE EXPRESSION
        yInfo() << "face expression: initial \n";
        yarp::os::Bottle face_expression_ini;

        face_expression_ini.addString("set");
        face_expression_ini.addString("all");
        face_expression_ini.addString("sur");
        portKinematicsFaceExpression.write(face_expression_ini);
        
        face_expression_ini.clear();
        face_expression_ini.addString("set");
        face_expression_ini.addString("eli");
        face_expression_ini.addString("hap");
        portKinematicsFaceExpression.write(face_expression_ini);


        Time::delay(5.0);


        // LOOK DOWN
        // look down to table
        yInfo() << "look_down: request";
        yarp::os::Bottle request_ld, response_ld;
        request_ld.addString("look_at");
        request_ld.addDouble(-30);
        rpcKinematicsHighFive.write(request_ld, response_ld);
        yInfo() << "look_down: finished \n";
        */

        //VISION and CLASSIFICATION

        // read input from vision
        yInfo() << "Reading...";
        Bottle *output = portVision.read();
        yInfo() << "Done!";

        yarp::os::Bottle *input_boxes = output->get(0).asList()->get(2).asList();//->write(boxes);
        int list_size = input_boxes->size();
        
        // TODO
        // Verifies that at least one object has been detected
        //if(list_size < 1){
        //    Bottle& exitStatus = exitStatusPort.prepare();
        //    bool status = false;
        //    output.addInt(status);
        //    exitStatusPort.write();
        //    yError() << "No objects detected!";
        //    return;
        //}

        std::vector<std::string> list_labels(list_size);
        for (int i = 0; i < list_size / 4; i++) {
            Bottle& output = portClassifierROI.prepare();
            Vector box;
            box(0) = input_boxes->get(i * 4).asInt();
            box(1) = input_boxes->get(i * 4 + 1).asInt();
            box(2) = input_boxes->get(i * 4 + 2).asInt();
            box(3) = input_boxes->get(i * 4 + 3).asInt();
            output.addList().read(box);
            portClassifierROI.write();

            yInfo() << "Classifier...";
            Bottle *input = portClassifierLabel.read();
            list_labels[i] = input->get(0).asString();
            yInfo() << "Done!";
        }       

        // process labels and get desired position
        yarp::os::Bottle *input_coord = output->get(1).asList()->get(2).asList();
        list_size = input_coord->size();
        Vector desired_position(3);
        for (unsigned n = 0; n < list_labels.size(); ++n) {
            //cout << list_labels.at( n ) << " ";
            if (desired_object == list_labels.at(n)) {
                desired_position(0) = input_coord->get(n * 3).asDouble();
                desired_position(1) = input_coord->get(n * 3 + 1).asDouble();
                desired_position(2) = input_coord->get(n * 3 + 2).asDouble();
                // TODO
                // Checks that the observed position is in a reacheable place
                //if(desired_position(0) > abs(0.7) || desired_position(1) > abs(0.7) || desired_position(3) > abs(0.7)){
                //    Bottle& exitStatus = exitStatusPort.prepare();
                //    bool status = false;
                //    output.addInt(status);
                //    exitStatusPort.write();
                //    yError() << "One of the world coordinates was out of range";
                //    return;
                //}  
            }
        }
        
        
        /*
        // POINT TO OBJECT
        yInfo() << "point to: request";
        yarp::os::Bottle request_pt, response_pt;
        request_pt.addString("point_to");
        request_pt.addDouble(-1.0);
        request_pt.addDouble(0.0);
        request_pt.addDouble(0.0);
        //request_pt.addDouble(desired_position(0));
        //request_pt.addDouble(desired_position(1));
        //request_pt.addDouble(desired_position(2));
        rpcKinematicsHighFive.write(request_pt, response_pt);
        yInfo() << "point to: finished \n";

        
        Time::delay(2.0);
        

        // HIGH FIVE
        yInfo() << "high-five: request";
        yarp::os::Bottle request_hf, response_hf;
        request_hf.addString("high_five");
        rpcKinematicsHighFive.write(request_hf, response_hf);
        yInfo() << "high-five: finished \n";


        Time::delay(2.0);


        // ASK FOR FEEDBACK
        yInfo() << "feedback: request";
        yarp::os::Bottle request_feed, response_feed;
        request_feed.addString("Give me feedback");
        rpcDynamicsFeedback.write(request_feed, response_feed);
        yInfo() << "feedback: finished \n";

        // REACT ACCORDING TO FEEDBACK
        yarp::os::Bottle face_expression;
        bool feedback = response_feed.get(0).asBool();
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
        */
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
        //exitStatusPort.interrupt();
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
        //exitStatusPort.close();
        return true;
    }
};

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    Manager manager;
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    yInfo()<<"Configure module...";
    manager.configure(rf);

    yInfo()<<"Start module...";
    manager.runModule();

    yInfo()<<"Main returning...";
    return 0;
}
