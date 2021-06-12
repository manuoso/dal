//---------------------------------------------------------------------------------------------------------------------
//  DJI ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2021 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
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

#pragma once

#include <vector>
#include <functional>

#include <djiosdk/dji_telemetry.hpp>
#include <djiosdk/dji_mfio.hpp>

namespace dal    {
namespace common {
namespace types  {

    using namespace DJI::OSDK;
    using namespace DJI::OSDK::Telemetry;

    const double C_EARTH = 6378137.0;
    const double C_PI = 3.141592653589793;

    struct Config
    {
        int app_id = 0;
        std::string app_key = "";
        bool useLog = false;
        bool useLogDJI = true;
        bool useAdvancedSensing = false;
        std::string device = "";
        unsigned int baudrate = 0;
        bool isM600 = false;
    };

    typedef std::vector<TopicName> Topics;
    /// TopicName element must be equal to:
    // TOPIC_POSITION_VO
    // TOPIC_GPS_FUSED
    // TOPIC_GPS_DETAILS
    // TOPIC_GPS_SIGNAL_LEVEL
    // TOPIC_ALTITUDE_FUSIONED
    // TOPIC_ANGULAR_RATE_FUSIONED
    // TOPIC_HARD_SYNC
    // TOPIC_COMPASS
    // TOPIC_QUATERNION
    // TOPIC_VELOCITY
    // TOPIC_STATUS_FLIGHT
    // TOPIC_STATUS_DISPLAYMODE
    // TOPIC_BATTERY_INFO
    // TOPIC_RC
    // TOPIC_RC_WITH_FLAG_DATA
    // TOPIC_RC_FULL_RAW_DATA
    // TOPIC_ANGULAR_RATE_RAW
    // TOPIC_ACCELERATION_RAW
    // TOPIC_CONTROL_DEVICE

    const Topics DefaultTopics400hz = {TOPIC_HARD_SYNC, TOPIC_ANGULAR_RATE_RAW, TOPIC_ACCELERATION_RAW};
    const Topics DefaultTopics200hz = {TOPIC_ANGULAR_RATE_FUSIONED, TOPIC_QUATERNION, TOPIC_VELOCITY, TOPIC_ALTITUDE_FUSIONED};
    const Topics DefaultTopics50hz = {TOPIC_GPS_FUSED, TOPIC_COMPASS, TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE, TOPIC_RC, TOPIC_RC_WITH_FLAG_DATA, TOPIC_RC_FULL_RAW_DATA};
    const Topics DefaultTopics5hz = {TOPIC_GPS_DETAILS, TOPIC_GPS_SIGNAL_LEVEL, TOPIC_BATTERY_INFO};
    
    struct dataMission{
        // Hotpoint config
        float               radiusHP            = 0.0;  // 5 - 500 m
        float               yawRateHP           = 0.0;  // 0 - 30 /s
        int                 clockWiseHP         = 0;    // 0 -> counter clockwise, 1-> clockwise
        int                 startPointHP        = 0;    // 0 -> North, 1 -> South, 2 -> West, 3 -> East, 4 -> Current pos to nearest on the hotpoint
        // Waypoint config
        float               maxVelWP            = 2.0;     
        float               idleVelWP           = 1.0;
        int                 finishActionWP      = 0;    // 0 -> no act, 1 -> return home, 2 -> landing, 3 -> return to point 0, 4 -> infinite mode
        int                 executiveTimesWP    = 0;    // 0 -> once, 1 -> twice
        int                 traceModeWP         = 0;    // 0 -> point by point, 1 -> smooth transition
        int                 rcLostWP            = 0;    // 0 -> exit, 1 -> continue
        int                 turnModeWP          = 0;    // 0 -> clockwise, 1 -> counter clockwise
        // Global config                                // Hotpoint: 0 -> point to velocity direction, 1 -> face inside, 2 -> face outside, 3 -> controlled by RC, 4 -> same starting
        int                 yawMode             = 0;    // Waypoint: 0 -> auto, 1 -> lock initial, 2 -> by RC, 3 -> waypoints yaw 
        std::string         missionType         = "";
    };

    typedef std::map<MFIO::CHANNEL, MFIO::MODE> Channels;
    /// CHANNELS:               | MODE:
    // CHANNEL_0    |   SDK1    | MODE_PWM_OUT  
    // CHANNEL_1    |   SDK2    | MODE_PWM_IN   
    // CHANNEL_2    |   SDK3    | MODE_GPIO_OUT 
    // CHANNEL_3    |   SDK4    | MODE_GPIO_IN 
    // CHANNEL_4    |   SDK5    | MODE_ADC 
    // CHANNEL_5    |   SDK6    | 
    // CHANNEL_6    |   SDK7    |
    // CHANNEL_7    |   SDK8    |

    typedef TypeMap<TOPIC_GPS_FUSED>::type              LatLon;
    typedef TypeMap<TOPIC_GPS_DETAILS>::type            GPSDetail;
    typedef TypeMap<TOPIC_GPS_SIGNAL_LEVEL>::type       GPSSignal;
    typedef TypeMap<TOPIC_ALTITUDE_FUSIONED>::type      Altitude;
    typedef TypeMap<TOPIC_ANGULAR_RATE_FUSIONED>::type  AngularRate;
    typedef TypeMap<TOPIC_ANGULAR_RATE_RAW>::type       AngularRateRaw;
    typedef TypeMap<TOPIC_ACCELERATION_RAW>::type       AccelerationRaw;
    typedef TypeMap<TOPIC_HARD_SYNC>::type              HardSync_FC;
    typedef TypeMap<TOPIC_COMPASS>::type                Compass;
    typedef TypeMap<TOPIC_QUATERNION>::type             Quaternion;
    typedef TypeMap<TOPIC_VELOCITY>::type               Velocity;
    typedef TypeMap<TOPIC_STATUS_FLIGHT>::type          FlightStatus;
    typedef TypeMap<TOPIC_STATUS_DISPLAYMODE>::type     Mode;
    typedef TypeMap<TOPIC_BATTERY_INFO>::type           Battery_info;
    typedef TypeMap<TOPIC_RC>::type                     RcBasic;
    typedef TypeMap<TOPIC_RC_WITH_FLAG_DATA>::type      Rc;
    typedef TypeMap<TOPIC_RC_FULL_RAW_DATA>::type       RcRaw;
    typedef TypeMap<TOPIC_CONTROL_DEVICE>::type         ControlDevice;
    typedef std::vector<float>                          LocalPoseGPS;

    typedef std::function<void(LatLon)>                 CallbackLatLon;
    typedef std::function<void(GPSDetail)>              CallbackGPSDetail;
    typedef std::function<void(GPSSignal)>              CallbackGPSSignal;
    typedef std::function<void(Altitude)>               CallbackAltitude;
    typedef std::function<void(AngularRate)>            CallbackAngRate;
    typedef std::function<void(AngularRateRaw)>         CallbackAngRateRaw;
    typedef std::function<void(AccelerationRaw)>        CallbackAccRaw;
    typedef std::function<void(HardSync_FC)>            CallbackIMU;
    typedef std::function<void(Compass)>                CallbackCompass;
    typedef std::function<void(Quaternion)>             CallbackQuat;
    typedef std::function<void(Velocity)>               CallbackVel;
    typedef std::function<void(FlightStatus)>           CallbackFlighStatus;
    typedef std::function<void(Mode)>                   CallbackMode;
    typedef std::function<void(Battery_info)>           CallbackBattery;
    typedef std::function<void(RcBasic)>                CallbackRcBasic;
    typedef std::function<void(Rc)>                     CallbackRc;
    typedef std::function<void(RcRaw)>                  CallbackRcRaw;
    typedef std::function<void(ControlDevice)>          CallbackControlDevice;
    typedef std::function<void(LocalPoseGPS)>           CallbackLocalPoseGPS;

}
}
}
