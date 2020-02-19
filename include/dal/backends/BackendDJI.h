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


#ifndef DAL_BACKENDS_BACKENDDJI_H_
#define DAL_BACKENDS_BACKENDDJI_H_

#include <dal/backends/Backend.h>  

// System Includes
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <mutex>
#include <vector>
#include <iostream>
#include <map>

// DJI OSDK includes
#include <djiosdk/dji_status.hpp>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>

// Logs
#include <dal/LogStatus.h>

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
    class BackendDJI: public Backend{
        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------

            /// Constructor
            BackendDJI();

            /// Destructor
            ~BackendDJI();

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR CONTROL
	        //---------------------------------------------------------------------------------------------------------------------

            /// This method is the implementation of emergency brake.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool emergencyBrake();

            /// This method is the implementation of recover control.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool recoverFromManual();

            /// This method is the implementation of takeoff using DJI SDK, this will block the main thread.
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool takeOff(const float _height);

            /// This method is the implementation of land using DJI SDK, this will block the main thread.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool land();

            /// This method is the implementation of position control and yaw using DJI SDK.
            /// \param _x: desired x in NEU coordinates (m).
            /// \param _y: desired y in NEU coordinates (m).
            /// \param _z: desired z in NEU coordinates (m).
            /// \param _yaw: desired yaw (deg).
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw);
	    
            /// This method is the implementation of velocity control and yaw using DJI SDK.
            /// \param _x: desired Vx in NEU coordinates (m/s).
            /// \param _y: desired Vy in NEU coordinates (m/s).
            /// \param _z: desired Vz in NEU coordinates (m/s).
            /// \param _yaw: desired yaw rate (deg/s).
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate);

            /// This method is the implementation of the attitude and vertical position control using DJI SDK.
            /// \param _roll: attitude set-point in x axis of body frame in FRU coordinates (deg).
            /// \param _pitch: attitude set-point in y axis of body frame FRU coordinates (deg).
            /// \param _yaw: attitude set-point in z axis of ground frame NED coordinates (deg).
            /// \param _z: position set-point in z axis of ground frame NED (m).
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool attitudeCtrlVer(float _roll, float _pitch, float _yaw, float _z);

            /// This method is the implementation of the attitude rate and vertical position control using DJI SDK.
            /// \param _rollRate: attitude rate set-point in x axis of body frame FRU coordinates (deg/s).
            /// \param _pitchRate: attitude rate set-point in y axis of body frame FRU coordinates (deg/s).
            /// \param _yawRate: attitude rate set-point in z axis of body frame FRU coordinates (deg/s).
            /// \param _z: z position set-point in z axis of ground frame NED coordinates (m).
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool angularRateCtrlVer(float _rollRate, float _pitchRate, float _yawRate, float _z);

            /// This method is the implementation of control using DJI SDK with custom flags.
            /// \param  uint8_t _flag:
            /// \param _xSP: 
            /// \param _ySP: 
            /// \param _zSP: 
            /// \param _yawSP:
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool selfControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP);

            /// This method is the implementation of move onepoint in GPS Coordinate using DJI SDK.
            /// \param _wayPoint: data with the GPS coordinates, where 0 = lat, 1 = lon, 2 = alt.
            /// \param _config: struct for configure the mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool positionCtrlGPS(Eigen::Vector3f _wayPoint, dataMission _config);

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR MISSIONS
	        //---------------------------------------------------------------------------------------------------------------------

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

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR TELEMETRY
	        //---------------------------------------------------------------------------------------------------------------------

            //---------------------------------------------------------------------------------------------------------------------
            // INITS

            /// This method is the implementation of init local position using GPS.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool initLocalPosition();

            //---------------------------------------------------------------------------------------------------------------------
            // GETTERS

            /// This method is the implementation of get local position VO.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getPositionVO(VectorPositionVO& _data);

            /// This method is the implementation of get GPS.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getGPS(VectorGPS& _data);

            /// This method is the implementation of get GPS Detail.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getGPSDetail(VectorGPSDetail& _data);

            /// This method is the implementation of get GPS Signal Level.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getGPSSignal(int& _data);

            /// This method is the implementation of get Altitude fusioned.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getAltitude(float& _data);

            /// This method is the implementation of get Angular Rate fusioned.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getAngularRate(VectorAngularRate& _data);

            /// This method is the implementation of get Hard Sync.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getHardSync(VectorHardSync& _data);

            /// This method is the implementation of get Compass.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getCompass(VectorCompass& _data);

            /// This method is the implementation of get Quaternion.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getQuaternion(VectorQuaternion& _data);

            /// This method is the implementation of get Velocity.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getVelocity(VectorVelocity& _data);

            /// This method is the implementation of get Status Flight.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getStatusFlight(std::string& _data);

            /// This method is the implementation of get Display Mode.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getDisplayMode(std::string& _data);

            /// This method is the implementation of get Batery Info.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getBatery(int& _data);

            /// This method is the implementation of get RC with Flag Data.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getRC(VectorRC& _data);

            /// This method is the implementation of get Control Device Info.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getControlDevice(VectorControlDevice& _data);

            /// This method is the implementation of get local position using GPS.
            /// \param _data: x, y and z returned (m).
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool getLocalPosition(VectorLocalPosition& _data);

        private:
            /// This method initialize DJI Vehicle and some important params.
            /// \param _config: needed configuration to initialize DJI SDK
            virtual bool init(const Config &_config);

            /// This method is the implementation of obtain the control of the vehicle.
            /// \return true if params are good or set without errors, false if something failed.
            bool obtainControlAuthority(bool _info);

            /// This method is the implementation of subscription to Telemetry topics.
            /// \param _topic: topic that we want to initialize to subscribe.
            /// \param _freq: frequency of the desired topic data.
            /// \return true if params are good or set without errors, false if something failed.
            bool subscribeToTopic(std::vector<DJI::OSDK::Telemetry::TopicName> _topics, int _freq);

            /// This method is the implementation of unsubscribscription of all data.
            /// \return true if params are good or set without errors, false if something failed.
            bool unsubscribeAllTopics();

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

        private:
            // Data for internal telemetry
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_POSITION_VO>::type            position_vo_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type              latLon_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS>::type            GPSDetail_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL>::type       GPSSignal_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type      altitude_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type  angularRate_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type                         hardSync_FC_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_COMPASS>::type                           compass_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type             quaternion_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_VELOCITY>::type               velocity_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>::type          flightStatus_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>::type     mode_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type                      battery_info_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA>::type      rc_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>::type         controlDevice_;
            DJI::OSDK::Telemetry::Vector3f                                                          localPose_;

        private:
            DJI::OSDK::Vehicle* vehicle_ = nullptr;

            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type originAltitude_, actualAltitude_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type originGPS_;

            std::mutex secureGuard_;

            bool useLogStatus_ = false;
            
            int pkgIndex_ = 0;
            std::string missionType_ = "";

            int functionTimeout_ = 1; // 666 TODO: WTF IS FUNCTIONTIMEOUT???
    };
}

#endif
