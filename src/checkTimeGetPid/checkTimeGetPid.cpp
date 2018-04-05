// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 iCub Facility
 * Authors: Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <cmath>

#include <rtf/dll/Plugin.h>
#include <rtf/TestAssert.h>
#include <yarp/rtf/TestAsserter.h>

#include "checkTimeGetPid.h"
#include "yarp/os/Bottle.h"
#include <yarp/dev/IPidControl.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace RTF;

using namespace yarp::os;
using namespace yarp::dev;


// prepare the plugin
PREPARE_PLUGIN(checkTimeGetPid)

checkTimeGetPid::checkTimeGetPid() : yarp::rtf::TestCase("checkTimeGetPid") 
{


}

checkTimeGetPid::~checkTimeGetPid() 
{
}

bool checkTimeGetPid::setup(yarp::os::Property &config)
{

    // initialization goes here ...

    iEncoders=NULL;
    iPidCtrl=NULL;
    initialized=false;
    numofAttempts=0;

    if(config.check("name"))
        setName(config.find("name").asString());
    else
        setName("checkTimeGetPid");
        
        
    // updating parameters
    RTF_ASSERT_ERROR_IF_FALSE(config.check("robot"),  "The robot name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(config.check("part"),   "The part name must be given as the test parameter!");
    RTF_ASSERT_ERROR_IF_FALSE(config.check("numberOfAttempts"),  "Number of attempts used to calculate the average tim of getPids");
    RTF_ASSERT_ERROR_IF_FALSE(config.check("delay"),   "delay in seconds between two concecutive attempts");
    
    robotName = config.find("robot").asString();
    partName  = config.find("part").asString();


    Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+robotName+"/"+partName);
    options.put("local", "/pidTimeTest/"+robotName+"/"+partName);

    dd = new yarp::dev::PolyDriver(options);
    RTF_ASSERT_ERROR_IF_FALSE(dd->isValid(),"Unable to open device driver");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(iPidCtrl),"Unable to open pid interface");
    RTF_ASSERT_ERROR_IF_FALSE(dd->view(iEncoders),"Unable to open encoder interface");
    
    numofAttempts = config.find("numberOfAttempts").asInt();
    elapseds = new double[numofAttempts];
    
    RTF_ASSERT_ERROR_IF_FALSE(numofAttempts>0,"numberOfAttempts not valid");

    delay = config.find("delay").asDouble();
    std::cout << "Delay = " << delay <<std::endl;
    RTF_ASSERT_ERROR_IF_FALSE(delay>0.005,"delay should be bigger than 0.005 sec");
    
    ;
    RTF_ASSERT_ERROR_IF_FALSE(iEncoders->getAxes(&nj),"fault in getAxes");
    
    if(config.check("file"))
        filename = config.find("file").asString();
    else
        filename = "out.txt";
    std::cout << "the test prints output on " << filename.c_str() << " file" << std::endl;
    
    return true;
}

void checkTimeGetPid::tearDown() {

    if(elapseds)
        delete elapseds;
    elapseds=nullptr;
    
}



void checkTimeGetPid::run() {

    double max=0.0; //sec
    double min = 100.0; //sec
    double tot=0.0;
    Pid pids[nj];
    double encs[nj];
    int errors=0;
    int spikes=0;
    double last_encs[nj];
    
    std::cout << "check validity...";

    Pid pidaux[nj];
    for(int j=0; j<nj; j++)
    {
        iPidCtrl->getPid(VOCAB_PIDTYPE_POSITION, j, &pidaux[j]);
        
    }
    
   
    iPidCtrl->getPids(VOCAB_PIDTYPE_POSITION, pids);
    for(int j=0; j<nj; j++)
    {
        if( (pids[j].kp != pidaux[j].kp) || 
            (pids[j].kd != pidaux[j].kd) ||
            (pids[j].ki != pidaux[j].ki) )
        {
            std::cout << "error!! on j " << j;
            RTF_ASSERT_ERROR("pid not equal!!");
        }
        
    }
    
    
    if(!iEncoders->getEncoders(last_encs))
        std::cout << "error in getEncoders";
    
    std::cout << "OK all pids are equal!! Start next step...";
    
    for(int i=0; i<numofAttempts; i++)
    {
        double start = yarp::os::Time::now();
        iPidCtrl->getPids(VOCAB_PIDTYPE_POSITION, pids);
        elapseds[i]= yarp::os::Time::now() - start;
        tot+=elapseds[i];
        if(elapseds[i]<min)
            min = elapseds[i];
        if(elapseds[i]> max)
            max = elapseds[i];
        
        for(int p=0; p<nj; p++)
            encs[p] = 200.0;
        
        bool ret = iEncoders->getEncoders(encs);
        if(!ret)
            std::cout << "error in getEncoders" <<std::endl;
        
        for(int j=0; j<nj; j++)
        {
            if(encs[j] == 0.0)
            {errors++;}
            
            if( ((last_encs[j] - encs[j]) >5) || ((last_encs[j] - encs[j]) < -5))
            {spikes++;}
        }
        
        
        for(int p=0; p<nj; p++)
            last_encs[p] = encs[p];
        
        yarp::os::Time::delay(delay);
    }
    
    //check getEncoders
    //if(errors>0)
        std::cout << "GET_ENCODERS ERROR = "<< errors << " SPIKES= " << spikes << std::endl;
        
    
    //print to file!!
    std::fstream fs;
    fs.open (filename.c_str(), std::fstream::out);
    
    fs << "--- checkTimeGetPid ---" << std::endl;
    std::string s = "Part = " + partName + " Number of attempts = " + std::to_string(numofAttempts) +  "; delay = " + std::to_string(delay);
    
    //fs << "Part = " << partName.c_str() << "Number of attempts = " << numofAttempts << "; delay = " << delay << std::endl;
    fs << s.c_str() << std::endl;
    
    fs << std::endl;
    
    fs << "Max = " << max << "; Min = " << min << "; Avg= " << tot/numofAttempts << std::endl;
    fs << std::endl;
    
    for(int i=0; i<numofAttempts; i++)
    {
        fs << i << "  " << elapseds[i] << std::endl;
    }
    
    fs.close();
    
    RTF_TEST_REPORT(s.c_str());

}
