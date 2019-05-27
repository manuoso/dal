//---------------------------------------------------------------------------------------------------------------------
//  DRONE ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 ViGUS University of Seville
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


#ifndef DAL_BACKENDS_BACKENDDJI_H_
#define DAL_BACKENDS_BACKENDDJI_H_

#include <dal/backends/Backend.h>  

// System Includes
#include <cmath>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <mutex>
#include <vector>
#include <iostream>

// DJI OSDK includes
#include <djiosdk/dji_status.hpp>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>

// Logs
#include <dal/LogStatus.h>
#include <dal/LogTelemetry.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

namespace dal{
    class BackendDJI: public Backend{
        public:
            /// Constructor
            BackendDJI();

            /// Destructor
            ~BackendDJI();

            /// This method is the implementation of takeoff using DJI SDK, this will block the main thread.
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool takeOff(const float _height);

            /// This method is the implementation of land using DJI SDK, this will block the main thread.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool land();

            /// This method is the implementation of emergency brake.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool emergencyBrake();

            /// This method is the implementation of recover control.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool recoverFromManual();

            /// This method is for configure a desired mission given the waypoints in GPS coordinates.
            /// \param _wayPoints: vector with the GPS coordinates of each point, where 0 = lat, 1 = lon, 2 = alt.
            /// If you select hotpoint, _wayPoints[0](0) will be the longitude, 
            /// _wayPoints[0](1) will be the latitude and _wayPoints[0](2) the altitude of the hotpoint.
            /// \param _config: struct for configure the mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config);

            /// This method is for start a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool start_mission();

            /// This method is for pause a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool pause_mission();

            /// This method is for stop a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool stop_mission();

            /// This method is for resume a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool resume_mission();

            /// This method is the implementation of position control and yaw using DJI SDK.
            /// \param _x: desired x in NEU coordinates.
            /// \param _y: desired y in NEU coordinates.
            /// \param _z: desired z in NEU coordinates.
            /// \param _yaw: desired yaw.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw);
	    
            /// This method is the implementation of velocity control and yaw using DJI SDK.
            /// \param _x: desired Vx in NEU coordinates.
            /// \param _y: desired Vy in NEU coordinates.
            /// \param _z: desired Vz in NEU coordinates.
            /// \param _yaw: desired yaw rate.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate);

            /// This method is the implementation of get Telemetry data.
            /// \param _data: struct with the desired received data.
            /// \param _saveToFile: if true save data received.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool receiveTelemetry(dataTelemetry& _data, bool _saveToFile);

        private:
            /// This method initialize DJI Vehicle and some important params.
            /// \param _config: needed configuration to initialize DJI SDK
            virtual bool init(const Config &_config);

            /// This method is the implementation of obtain the control of the vehicle.
            /// \return true if params are good or set without errors, false if something failed.
            bool obtainControlAuthority(bool _info);

            /// This method is the implementation of subscription to Telemetry data and others.
            /// By defect, we will subscribe to six kinds of data:
            /// 1. Flight Status at 10 Hz
            /// 2. Mode at 10 Hz
            /// 2. Fused Lat/Lon at 50Hz
            /// 3. Fused Altitude at 50Hz
            /// 4. RC Channels at 50 Hz
            /// 5. Velocity at 50 Hz
            /// 6. Quaternion at 200 Hz
            /// 7. RTK if available at 5 Hz
            /// \return true if params are good or set without errors, false if something failed.
            bool subscribeToData();

            /// This method is the implementation of unsubscribscription of all data.
            /// \return true if params are good or set without errors, false if something failed.
            bool unsubscribeToData();

            /// This method is the implementation of set the local position to move the vehicle in local coordinates.
            /// \return true if params are good or set without errors, false if something failed.
            bool setLocalPosition();

            /// This method is the implementation of a very simple calculation of local NED offset between two pairs of GPS coordinates. Accurate when distances are small.
            /// \param _delta: offset returned in NED coordinates.
            /// \param _target: target position in GPS coordinates.
            /// \param _origin: origin position in GPS coordinates.
            void localPoseFromGps(DJI::OSDK::Telemetry::Vector3f& _delta, void* _target, void* _origin);

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

            /// 666 TODO: NEED TO CHECK!!!
            /// This method convert a quaternion to Euler Angle.
            /// \param _quaternionData: quaternion to convert.
            /// \return the converted result.
            DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* _quaternionData);

            /// This method is the implementation of start global broadcast of all data.
            /// \return true if params are good or set without errors, false if something failed.
            bool startGlobalPositionBroadcast();

        private:
            DJI::OSDK::Vehicle* vehicle_;
            DJI::OSDK::Vehicle::ActivateData activateData_;

            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type originGPS_;
            DJI::OSDK::Telemetry::GlobalPosition broadcastGP_;

            std::mutex secureGuard_;

            // RTK can be detected as unavailable only for Flight controllers that don't support RTK
            bool rtkAvailable_ = false;

            int pkgIndex_ = 0;
            std::string missionType_ = "";

            int functionTimeout_ = 1; // 666 TODO: WTF IS FUNCTIONTIMEOUT???
    };
}

#endif
