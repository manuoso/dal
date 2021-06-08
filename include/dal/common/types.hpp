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

namespace dal    {
namespace common {
namespace types  {

    const double C_EARTH = 6378137.0;
    const double C_PI = 3.141592653589793;

    struct Config
    {
        /// ID of DJI APP
        int app_id;
        
        /// KEY of DJI APP
        std::string app_key = "";

        /// Use logging
        bool useLog = false;

        /// Use DJI logging
        bool useLogDJI = true;

        /// Use advance sensing for DJI
        bool useAdvancedSensing = false;

        /// Device port of the controller
        std::string device = "";

        /// Baudrate to connect to the controller
        unsigned int baudrate;

        /// Is DJI Matrice 600
        bool isM600 = false;
    };

    typedef std::map<DJI::OSDK::Telemetry::TopicName, int> Topics;
    // TopicName element must be equal to:
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
    
}
}
}
