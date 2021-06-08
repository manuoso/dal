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

#include <dal/hal.hpp>

namespace dal     {
namespace modules {

    class Missions
    {
        public:
            Missions(std::shared_ptr<HAL> & _hal);
            ~Missions();

        private:
            std::shared_ptr<HAL> hal_;
    };
    
}
}

/*
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// ¡¡¡ IMPORTANT !!! 
//
// This backend is developed for the DJI A3 controller. 
// So the implemented functions may vary for another model like the M210 and M600.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

namespace dal{
    class MissionsDJI{
        public:
            /// Struct that we use to configure a Mission
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

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------

            /// Constructor
            MissionsDJI();

            /// Destructor
            ~MissionsDJI();

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR MISSIONS
	        //---------------------------------------------------------------------------------------------------------------------

            /// This method is the implementation of move onepoint in GPS Coordinate using DJI SDK.
            /// \param _wayPoint: data with the GPS coordinates, where 0 = lat, 1 = lon, 2 = alt.
            /// \param _config: struct for configure the mission.
            /// \return true if params are good or set without errors, false if something failed.
            bool positionGPS(Eigen::Vector3f _wayPoint, dataMission _config);
            
            /// This method is for configure a desired mission given the waypoints in GPS coordinates.
            /// \param _wayPoints: vector with the GPS coordinates of each point, where 0 = lat, 1 = lon, 2 = alt.
            /// If you select hotpoint, _wayPoints[0][0] will be the longitude, 
            /// _wayPoints[0][1] will be the latitude and _wayPoints[0][2] the altitude of the hotpoint.
            /// \param _config: struct for configure the mission.
            /// \return true if params are good or set without errors, false if something failed.
            bool mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config);

            /// This method is for start a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            bool start_mission();

            /// This method is for pause a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            bool pause_mission();

            /// This method is for stop a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            bool stop_mission();

            /// This method is for resume a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            bool resume_mission();

        private:
            /// This method defaults the waypoint options.
            /// \param _wp: waypoint to put by default.
            void setWaypointDefaults(DJI::OSDK::WayPointSettings* _wp);

            /// This method defaults the init waypoint options for DJI function.
            /// \param _wp: waypoint to put by default.
            void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* _fdata);

            /// This method generates a list of waypoints for the mission of DJI given a list of waypoints in GPS coordinates.
            /// \param _wayPoints: list of waypoints.
            /// \param _config: struct to configure each waypoint
            /// \return the waypoints converted.
            std::vector<DJI::OSDK::WayPointSettings> createWaypoints(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config);

            /// This method upload a list of waypoints for the mission of DJI given a list of waypoints in DJI type.
            /// \param _wpList: list of waypoints to upload.
            void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& _wpList);

        private:
            std::string missionType_ = "";

    };
}

*/