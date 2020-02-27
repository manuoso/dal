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


#ifndef DAL_HAL_H_
#define DAL_HAL_H_

// System Includes
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <mutex>
#include <vector>
#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <cassert>

// External libraries
#include <Eigen/Eigen>

// DJI OSDK includes
#include <djiosdk/dji_status.hpp>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>

// Modules
#include <dal/dji/ControlDJI.h>
#include <dal/dji/MissionsDJI.h>
#include <dal/dji/TelemetryDJI.h>
#include <dal/local_control/localControl.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// ¡¡¡ IMPORTANT !!! 
//
// This backend is developed for the DJI A3 controller. 
// So the implemented functions may vary for another model like the M210 and M600.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

namespace dal{
    class HAL{
        public:
            /// Struct for configure the backend
            struct Config{
                /// ID of DJI APP
                int app_id;
                
                /// KEY of DJI APP
                std::string app_key = "";

                /// Use logging
                bool useLog = false;

                /// Use advance sensing for DJI
                bool useAdvancedSensing = false;

                /// Device port of the controller
                std::string device = "";

                /// Baudrate to connect to the controller
                unsigned int baudrate;

                /// Map that contains Topics of Telemetry to use
                std::map<DJI::OSDK::Telemetry::TopicName, int> topics;
                // TopicName element of topics must be equal to:
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
                // DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA
                // DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE

            };

        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------
            
            /// Constructor
            HAL();

            /// Destructor
            ~HAL();

            /// This method initialize DJI Vehicle and some important params.
            /// \param _config: needed configuration to initialize DJI SDK
            /// \return true if params are good or set without errors, false if something failed.
            bool create(const Config &_config);

            /// Close the system.
            void close();

        private:
            //---------------------------------------------------------------------------------------------------------------------
            // UTILS
	        //---------------------------------------------------------------------------------------------------------------------

            /// This method is the implementation of obtain the control of the vehicle.
            /// \return true if params are good or set without errors, false if something failed.
            bool obtainControlAuthority();

            /// This method is the implementation of extract and configure topics from config.
            /// \return true if params are good or set without errors, false if something failed.
            bool extractTopics(std::map<DJI::OSDK::Telemetry::TopicName, int> _topics);

            /// This method is the implementation of set the local position to move the vehicle in local coordinates.
            /// \return true if params are good or set without errors, false if something failed.
            bool setLocalPosition();

            /// This method is the implementation of subscription to Telemetry topics.
            /// \param _topic: topic that we want to initialize to subscribe.
            /// \param _freq: frequency of the desired topic data.
            /// \return true if params are good or set without errors, false if something failed.
            bool subscribeToTopic(std::vector<DJI::OSDK::Telemetry::TopicName> _topics, int _freq);

            /// This method is the implementation of unsubscribscription of all data.
            /// \return true if params are good or set without errors, false if something failed.
            bool unsubscribeAllTopics();
            
        public:
            static DJI::OSDK::Vehicle* vehicle_;

            static int functionTimeout_; // 666 TODO: WTF IS FUNCTIONTIMEOUT???

            static DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type originAltitude_;
            static DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type originGPS_;

        private:
            int pkgIndex_ = 0;

    };

}

#endif
