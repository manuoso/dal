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


#ifndef DAL_H_
#define DAL_H_

#include <dal/backends/Backend.h>
#include <dal/PID.h>

#include <string>
#include <mutex>
#include <math.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cassert>
#include <Eigen/Eigen>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

namespace dal{
    class DAL {
        public:
            /// Especials Typedefs
            typedef Eigen::Matrix<float, 14, 1> VectorPID;

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------
            
            /// Initialize the system. 
            static void init(const Backend::Config &_config);

            /// Close the system.
            static void close();

            /// Get current instance of the system.
            static DAL* get();
        
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR CONTROL
	        //---------------------------------------------------------------------------------------------------------------------

            /// Method for emergency brake
            bool emergencyBrake();

            /// Method for recover control
            bool recoverFromManual();

            /// Method for take off
            bool takeOff(const float _height);

            /// Method for land
            bool land();

            /// Method for go to desire position using own function
            bool position(float _x, float _y, float _z, float _yaw);

            /// Method for move with desire velocity using own function
            bool velocity(float _vx, float _vy, float _vz, float _yawRate);

            /// Method for move in attitude using own function
            bool attitude(float _roll, float _pitch, float _yaw, float _z);

            /// Method for move in attitude rate using own function
            bool attitudeRate(float _rollRate, float _pitchRate, float _yawRate, float _z);
            
            /// Method for go to desire position using GPS coordinates
            bool positionGPS(Eigen::Vector3f _wayPoint, Backend::dataMission _config);

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR LOCAL CONTROL
	        //---------------------------------------------------------------------------------------------------------------------

            /// Method for initialize PID's
            bool initPID(std::string _type, VectorPID _x, VectorPID _y, VectorPID _z);

            /// Method for control the UAV using a PID
            Eigen::Vector4f localControl(float _x, float _y, float _z, float _vx, float _vy, float _yaw);

            /// Method for set the position of reference
            bool setReferencePIDV(float _x, float _y, float _z);

            /// Method for change Kp of the desired PID
            bool setKpPID(float _kp, std::string _pid);

            /// Method for change Ki of the desired PID
            bool setKiPID(float _ki, std::string _pid);

            /// Method for change Kd of the desired PID
            bool setKdPID(float _kd, std::string _pid);
            
            /// Method for convert from velocities to attitude commands
            Eigen::Vector4f convertAttiCommands(float _ax, float _ay, float _z, float _yaw);
             
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR MISSIONS
	        //---------------------------------------------------------------------------------------------------------------------

            /// Method for configure a desired mission given the waypoints
            bool mission(std::vector<Eigen::Vector3f> _wayPoints, Backend::dataMission _config);

            /// Method for start a configured mission
            bool start_mission();

            /// Method for pause a configured mission
            bool pause_mission();

            /// Method for stop a configured mission
            bool stop_mission();

            /// Method for resume a configured mission
            bool resume_mission();

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR TELEMETRY
	        //---------------------------------------------------------------------------------------------------------------------

            //---------------------------------------------------------------------------------------------------------------------
            // INITS

            /// Method for init Local Position
            bool initializeLocalPosition();

	        //---------------------------------------------------------------------------------------------------------------------
            // GETTERS
            
            /// Method for get telemetry local position VO
            bool telemetryPositionVO(Backend::VectorPositionVO& _data);

            /// Method for get telemetry GPS
            bool telemetryGPS(Backend::VectorGPS& _data);

            /// Method for get telemetry GPS Detail
            bool telemetryGPSDetail(Backend::VectorGPSDetail& _data);

            /// Method for get telemetry GPS Signal Level
            bool telemetryGPSSignal(int& _data);

            /// Method for get telemetry Altitude fusioned
            bool telemetryAltitude(float& _data);

            /// Method for get telemetry Angular Rate fusioned
            bool telemetryAngularRate(Backend::VectorAngularRate& _data);

            /// Method for get telemetry Hard Sync
            bool telemetryHardSync(Backend::VectorHardSync& _data);

            /// Method for get telemetry Compass
            bool telemetryCompass(Backend::VectorCompass& _data);

            /// Method for get telemetry Quaternion
            bool telemetryQuaternion(Backend::VectorQuaternion& _data);

            /// Method for get telemetry Velocity
            bool telemetryVelocity(Backend::VectorVelocity& _data);

            /// Method for get telemetry Status Flight
            bool telemetryStatusFlight(std::string& _data);

            /// Method for get telemetry Display Mode
            bool telemetryDisplayMode(std::string& _data);

            /// Method for get telemetry Batery Info
            bool telemetryBatery(int& _data);

            /// Method for get telemetry RC with Flag Data
            bool telemetryRC(Backend::VectorRC& _data);

            /// Method for get telemetry Control Device Info
            bool telemetryControlDevice(Backend::VectorControlDevice& _data);

            /// Method for get Local Position
            bool localPosition(Backend::VectorLocalPosition& _data);

        public:
            Backend * backend(){return backend_;}

        private:
            /// Constructor with given configuration for backend
            DAL(const Backend::Config &_config);

            /// Destructor
            virtual ~DAL(); 

        private:
		    static DAL *singleton_;

            Backend *backend_;

            std::mutex secureGuard_;
            
            std::string pidType_ = "";
            PID *pidVX_, *pidVY_, *pidVZ_, *pidAX_, *pidAY_;
            bool pidInitialized_ = false;
            float incT_ = 0;

            std::chrono::high_resolution_clock::time_point t0_, t1_;
            float refz_ = 0.0;

    };
}

#endif
