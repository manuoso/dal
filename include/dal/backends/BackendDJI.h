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

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

namespace dal{
    class BackendDJI: public Backend{
        public:
            /// Default constructor
            BackendDJI():Backend(){}

            /// This method is the implementation of takeoff using DJI SDK, this will block the main thread.
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool takeOff(const float _height);

            /// This method is the implementation of land using DJI SDK, this will block the main thread.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool land();

            /// This method is the implementation of move to position using DJI SDK.
            /// \param _x: desired x in NEU coordinates.
            /// \param _y: desired y in NEU coordinates.
            /// \param _z: desired z in NEU coordinates.
            /// \param _yaw: desired yaw.
            /// \param _posThreshold: position threshold in the desired position in Meters.
            /// \param _yawThreshold: yaw threshold in the desired position in Deg.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool movePosition(float _x, float _y, float _z, float _yaw, float _posThreshold = 0.2, float _yawThreshold = 1.0);

            /// This method is the implementation of position control and yaw using DJI SDK.
            /// \param _x: desired x in NEU coordinates.
            /// \param _y: desired y in NEU coordinates.
            /// \param _z: desired z in NEU coordinates.
            /// \param _yaw: desired yaw.
            /// \param _offset: calculate the position with offset or without a offset, that is local position in frame(local) or global.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw, bool _offset);
	    
            /// This method is the implementation of velocity control and yaw using DJI SDK.
            /// \param _x: desired Vx in NEU coordinates.
            /// \param _y: desired Vy in NEU coordinates.
            /// \param _z: desired Vz in NEU coordinates.
            /// \param _yaw: desired yaw rate.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate);

            /// This method is the implementation of get Telemetry data.
            /// \param _data: struct with the desired received data.
            /// \param _printData: if true print data received.
            /// \param _saveToFile: if true save data received.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool receiveTelemetry(dataTelemetry& _data, bool _printData, bool _saveToFile);

            /// This method is the implementation of mission with waypoints to do a polygon.
            /// \param _numWaypoints: number of desired points of the polygon.
            /// \param _increment: increment of each step in the mision.
            /// \param _startAlt: altitude of start.
            /// \return true if params are good or set without errors, false if something failed.
            bool runWaypointMissionPolygon(uint8_t _numWaypoints, float64_t _increment, float64_t _startAlt);

            /// This method is the implementation of mission with hot point to do a radius.
            /// \param _initialRadius: center of the desired radius.
            /// \param _time: time to do the circle.
            /// \return true if params are good or set without errors, false if something failed.
            bool runHotpointMissionRadius(int _initialRadius, int _time);

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

            /// This method is the implementation of a very simple calculation of local NED and ENU offset between two pairs of GPS coordinates. Accurate when distances are small..
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            void localOffsetFromGpsOffset(DJI::OSDK::Telemetry::Vector3f& _deltaNed, DJI::OSDK::Telemetry::Vector3f& _deltaEnu, void* _target, void* _origin);

            void setWaypointDefaults(DJI::OSDK::WayPointSettings* _wp);

            void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* _wp);

            std::vector<DJI::OSDK::WayPointSettings> createWaypoints(int _numWaypoints, DJI::OSDK::float64_t _distanceIncrement, DJI::OSDK::float32_t _startAlt);

            std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(DJI::OSDK::WayPointSettings* _startData, DJI::OSDK::float64_t _increment, int _numWp);

            void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& _wpList);

            DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* _quaternionData);

            bool startGlobalPositionBroadcast();

        private:
            DJI::OSDK::Vehicle* mVehicle;
            DJI::OSDK::Vehicle::ActivateData mActivateData;

            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type mOriginGPS;
            DJI::OSDK::Telemetry::GlobalPosition mBroadcastGP;

            std::mutex mSecureGuard;

            // RTK can be detected as unavailable only for Flight controllers that don't support RTK
            bool mRTKAvailable = false;

            int mPkgIndex = 0;

            int mFunctionTimeout = 1; // 666 TODO: WTF IS FUNCTIONTIMEOUT???
    };
}

#endif
