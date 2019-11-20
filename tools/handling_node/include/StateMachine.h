//---------------------------------------------------------------------------------------------------------------------
//  DJI ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <signal.h>

#include <dal/dal.h>

#include <fastcom/Subscriber.h>
#include <fastcom/Publisher.h>
#include <fastcom/ServiceServer.h>

#include <rapidjson/document.h>

#define CONST_PI (double)3.141592653589793
#define DEG_RAD(DEG) ((DEG) * ((CONST_PI) / (180.0)))
#define RAD_DEG(RAD) ((RAD) * (180.0) / (CONST_PI))

class StateMachine
{
  public:
    /// Init 
    /// \param _argc: argc from main
    /// \param _argv: argv from main
    bool init(int _argc, char** _argv);

    /// Run the state machine
    bool run();

    /// Finish cleanly the state machine
    bool finalize();

  private:
    /// Init publishers and subscribers of Fastcom
    bool initFastcom();

    /// Get telemetry and send it by publishers
    bool telemetryThread();

  private:
    /// States of the state machine
    enum class eState
    {   
        WAIT,
        LAND,
        TAKEOFF,
        BRAKE,
        MOVE_POS,
        MOVE_VEL,
        MOVE_ATT,
        MOVE_ATTR,
        RECOVER_CONTROL,
        EXIT
    };

    /// Struct of xyz
    struct xyz_data{   
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float yaw = 0.0;
    };

    /// Struct of xyz
    struct rpy_data{   
        float roll = 0.0;
        float pitch = 0.0;
        float yaw = 0.0;
        float z = 0.0;
    };

    /// Struct of GPS
    struct gps_data{   
        float latitude = 0.0;
        float longitude = 0.0;
        float altitude = 0.0;
    };

    /// Struct of RC Commands
    struct rc_data{   
        float rc1 = 0.0;
        float rc2 = 0.0;
        float rc3 = 0.0;
        float rc4 = 0.0;
        float flag1 = 0.0;
        float flag2 = 0.0;
        float flag3 = 0.0;
        float flag4 = 0.0;
    };

    struct RequestCommand{
        SERVICE_MESSAGE_TYPE
        int command;
    };

    struct ResponseError{
        SERVICE_MESSAGE_TYPE
        int error;
    };

  private:
    eState state_;

    rapidjson::Document configFile_;

    std::string path_ = "";

    fastcom::Subscriber<xyz_data> *subPose_ = nullptr;
    fastcom::Subscriber<xyz_data> *subVel_ = nullptr;
    fastcom::Subscriber<rpy_data> *subAtt_ = nullptr;
    fastcom::Subscriber<rpy_data> *subAttR_ = nullptr;

    fastcom::Publisher<char[20]> *pubState_ = nullptr;

    // Telemetry publishers
    fastcom::Publisher<char[30]> *pubMode_ = nullptr;
    fastcom::Publisher<char[30]> *pubFligStatus_ = nullptr;
    fastcom::Publisher<rc_data> *pubRC_ = nullptr;
    fastcom::Publisher<gps_data> *pubPosGPS_ = nullptr;
    fastcom::Publisher<xyz_data> *pubVel_ = nullptr;
    fastcom::Publisher<float> *pubBatLevel_ = nullptr;

    fastcom::ServiceServer<RequestCommand, ResponseError> *serverHandler_;

    std::thread telemThread_;

    xyz_data targetPose_, targetVelLinear_;
    rpy_data targetAtt_, targetAttR_;

    bool fin_ = false;
};
