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

#include <yarp/os/Vocab.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/***************************************************/
ObjectRetriever::ObjectRetriever()
{
    portLocation.open("/location");
    portCalibration.open("/calibration");

    portLocation.asPort().setTimeout(1.0);
    portCalibration.asPort().setTimeout(1.0);
}

/***************************************************/
ObjectRetriever::~ObjectRetriever()
{
    portLocation.close();
    portCalibration.close();
}

/***************************************************/
bool ObjectRetriever::calibrate(Vector &location)
{
    if ((portCalibration.getOutputCount()>0) &&
        (location.length()>=3))
    {
        Bottle cmd,reply;
        cmd.addString("get_location_nolook");
        cmd.addString("iol-right");
        cmd.addDouble(location[0]);
        cmd.addDouble(location[1]);
        cmd.addDouble(location[2]);
        portCalibration.write(cmd,reply);

        location.resize(3);
        location[0]=reply.get(1).asDouble();
        location[1]=reply.get(2).asDouble();
        location[2]=reply.get(3).asDouble();
        return true;
    }

    return false;
}

/***************************************************/
bool ObjectRetriever::getLocation(Vector &location)
{
    if (portLocation.getOutputCount()>0)
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content=cmd.addList().addList();
        content.addString("name");
        content.addString("==");
        content.addString("Toy");
        portLocation.write(cmd,reply);

        if (reply.size()>1)
        {
            if (reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *idField=reply.get(1).asList())
                {
                    if (Bottle *idValues=idField->get(1).asList())
                    {
                        int id=idValues->get(0).asInt();

                        cmd.clear();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &content=cmd.addList();
                        Bottle &list_bid=content.addList();
                        list_bid.addString("id");
                        list_bid.addInt(id);
                        Bottle &list_propSet=content.addList();
                        list_propSet.addString("propSet");
                        Bottle &list_items=list_propSet.addList();
                        list_items.addString("position_3d");
                        Bottle replyProp;
                        portLocation.write(cmd,replyProp);

                        if (replyProp.get(0).asVocab()==Vocab::encode("ack"))
                        {
                            if (Bottle *propField=replyProp.get(1).asList())
                            {
                                if (Bottle *position_3d=propField->find("position_3d").asList())
                                {
                                    if (position_3d->size()>=3)
                                    {
                                        location.resize(3);
                                        location[0]=position_3d->get(0).asDouble();
                                        location[1]=position_3d->get(1).asDouble();
                                        location[2]=position_3d->get(2).asDouble();
                                        if (calibrate(location))
                                            return true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    yError()<<"Unable to retrieve location";
    return false;
}
