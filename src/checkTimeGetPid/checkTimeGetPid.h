// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _MOVEMENTREFERNCESTEST_
#define _MOVEMENTREFERNCESTEST_

#include <yarp/rtf/TestCase.h>

#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include "yarp/rtf/JointsPosMotion.h"
#include "yarp/dev/IPidControl.h"

/**
* \ingroup icub-tests
* Check IPositionControl2, IVelocityControl2, IPWMControl, IPositionDirect.
*
* Check the following functions:
* \li IPositionControl2::getPositionTarget()
* \li IVelocityControl2::getRefVelocity()
* \li IPositionDirect::getRefPosition()
* \li IPWMControl::getRefDutyCycle()
*
*
*  Accepts the following parameters:
* | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | name           | string | -     | "checkTimeGetPid" | No       | The name of the test. | -     |
* | part           | string | -     | -             | Yes      | The yarp port name of the controlboard to test. | - |
* | numofAttempts  | int    | -     | -             | Yes      | Number of attempts used to calculate the average tim of "getPids" |  |
* | delay          | double | -     | -             | Yes      | delay in seconds between two concecutive attempts | |
* | file           | double | -     | -             | No       | name of output file | |
* 
* 
*/
class checkTimeGetPid : public yarp::rtf::TestCase {
public:
    checkTimeGetPid();
    virtual ~checkTimeGetPid();

    virtual bool setup(yarp::os::Property& configuration);

    virtual void tearDown();

    virtual void run();

private:
    void setAndCheckControlMode(int j, int mode);


    yarp::dev::PolyDriver *dd;
    yarp::dev::IEncoders *iEncoders;
    yarp::dev::IPidControl *iPidCtrl;
    
    int numofAttempts;
    double *elapseds;
    double delay;
    int nj;
    
    bool initialized;
    std::string robotName;
    std::string partName;
    std::string filename;

};

#endif //_MOVEMENTREFERNCESTEST_
