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

#include <map>

#include <djiosdk/dji_telemetry.hpp>
#include <djiosdk/dji_mfio.hpp>

namespace dal    {
namespace common {
namespace types  {

    using namespace DJI::OSDK;
    
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

    typedef std::map<DJI::OSDK::Telemetry::TopicName, int> Topics;
    /// TopicName element must be equal to:
    // DJI::OSDK::Telemetry::TOPIC_POSITION_VO
    // DJI::OSDK::Telemetry::TOPIC_GPS_FUSED
    // DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS
    // DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL
    // DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED
    // DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED
    // DJI::OSDK::Telemetry::TOPIC_HARD_SYNC
    // DJI::OSDK::Telemetry::TOPIC_COMPASS
    // DJI::OSDK::Telemetry::TOPIC_QUATERNION
    // DJI::OSDK::Telemetry::TOPIC_VELOCITY
    // DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT
    // DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE
    // DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO
    // DJI::OSDK::Telemetry::TOPIC_RC
    // DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA
    // DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA
    // DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_RAW
    // DJI::OSDK::Telemetry::TOPIC_ACCELERATION_RAW
    // DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE
    
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

}
}
}
