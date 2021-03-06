/*
 * iCub Robot Unit Tests (Robot Testing Framework)
 *
 * Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _GAZECONTROLSIMPLELOOK_H_
#define _GAZECONTROLSIMPLELOOK_H_

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>

/**
* \ingroup icub-tests
*
* This test verifies the simple gaze movements.
*
* Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required |  Description  | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-------------:|:-----:|
* |       -        |    -   |   -   |      -        |    -     |       -       |   -   |
*/
class GazeControlSimpleLookTest : public yarp::robottestingframework::TestCase
{
    yarp::dev::PolyDriver driver;

public:
    GazeControlSimpleLookTest();
    virtual ~GazeControlSimpleLookTest();
    virtual bool setup(yarp::os::Property& property);
    virtual void tearDown();
    virtual void run();
};

#endif
