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

#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>


class ObjectRetriever
{
    yarp::os::RpcClient portLocation;
    yarp::os::RpcClient portCalibration;
    bool calibrate(yarp::sig::Vector &location);

public:
    ObjectRetriever();
    bool getLocation(yarp::sig::Vector &location);
    virtual ~ObjectRetriever();
};
