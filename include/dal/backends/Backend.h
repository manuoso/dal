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


#ifndef DAL_BACKENDS_BACKEND_H_
#define DAL_BACKENDS_BACKEND_H_

#include <string>
#include <map>
#include <Eigen/Eigen>

#include <djiosdk/dji_telemetry.hpp>

namespace dal{
    class Backend{
        public:
            /// Especials Typedefs
            typedef Eigen::Matrix<float, 6, 1> VectorPositionVO;
            typedef Eigen::Matrix<float, 2, 1> VectorGPS;
            typedef Eigen::Matrix<float, 7, 1> VectorGPSDetail;
            typedef Eigen::Matrix<float, 3, 1> VectorAngularRate;
            typedef Eigen::Matrix<float, 10, 1> VectorHardSync;
            typedef Eigen::Matrix<float, 8, 1> VectorRC;
            typedef Eigen::Matrix<float, 3, 1> VectorCompass;
            typedef Eigen::Matrix<float, 4, 1> VectorQuaternion;
            typedef Eigen::Matrix<float, 3, 1> VectorVelocity;
            typedef Eigen::Matrix<int, 3, 1> VectorControlDevice;
            typedef Eigen::Matrix<float, 3, 1> VectorLocalPosition;

            /// Struct for configure the backend
            struct Config{
                /// Type of backend
                enum class eType {DJI, Dummy};
                eType type;

                /// ID of DJI APP
                int app_id;
                
                /// KEY of DJI APP
                std::string app_key = "";

                /// Use logging
                bool useLogStatus = false;

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

            /// Structs that we use to configure a Mission
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

        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------
            
            Backend();

            virtual ~Backend();

            static Backend* create(const Config &_config);

        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR CONTROL
	        //---------------------------------------------------------------------------------------------------------------------

            /// \brief abstract method for emergency brake
            virtual bool emergencyBrake() = 0;

            /// \brief abstract method for recover control
            virtual bool recoverFromManual() = 0;

            /// \brief abstract method for take off
            virtual bool takeOff(const float _height) = 0;

            /// \brief abstract method for land
            virtual bool land() = 0;

            /// \brief abstract method for position control and yaw
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw) = 0;
	    
            /// \brief abstract method for velocity control and yaw
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate) = 0;

            /// \brief abstract method for attitude control and vertical position
            virtual bool attitudeCtrlVer(float _roll, float _pitch, float _yaw, float _z) = 0;

            /// \brief abstract method for attitude rate control and vertical position
            virtual bool angularRateCtrlVer(float _rollRate, float _pitchRate, float _yawRate, float _z) = 0;

            /// \brief abstract method for with custom flags
            virtual bool selfControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP) = 0;

            /// \brief abstract method for position GPS
            virtual bool positionCtrlGPS(Eigen::Vector3f _wayPoint, dataMission _config) = 0;

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR MISSIONS
	        //---------------------------------------------------------------------------------------------------------------------

            /// \brief abstract method for configure a desired mission given the waypoints
            virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config) = 0;

            /// \brief abstract method for start a configured mission
            virtual bool start_mission() = 0;

            /// \brief abstract method for pause a configured mission
            virtual bool pause_mission() = 0;

            /// \brief abstract method for stop a configured mission
            virtual bool stop_mission() = 0;

            /// \brief abstract method for resume a configured mission
            virtual bool resume_mission() = 0;

	        //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR TELEMETRY
	        //---------------------------------------------------------------------------------------------------------------------
            
            //---------------------------------------------------------------------------------------------------------------------
            // INITS

            /// \brief abstract method for init Local Position
            virtual bool initLocalPosition() = 0;

            //---------------------------------------------------------------------------------------------------------------------
            // GETTERS

            /// \brief abstract method for receive local position VO
            virtual bool getPositionVO(VectorPositionVO& _data) = 0;

            /// \brief abstract method for receive GPS
            virtual bool getGPS(VectorGPS& _data) = 0;

            /// \brief abstract method for receive GPS Detail
            virtual bool getGPSDetail(VectorGPSDetail& _data) = 0;

            /// \brief abstract method for receive GPS Signal Level
            virtual bool getGPSSignal(int& _data) = 0;

            /// \brief abstract method for receive Altitude fusioned
            virtual bool getAltitude(float& _data) = 0;

            /// \brief abstract method for receive Angular Rate fusioned
            virtual bool getAngularRate(VectorAngularRate& _data) = 0;

            /// \brief abstract method for receive Hard Sync
            virtual bool getHardSync(VectorHardSync& _data) = 0;

            /// \brief abstract method for receive Compass
            virtual bool getCompass(VectorCompass& _data) = 0;

            /// \brief abstract method for receive Quaternion
            virtual bool getQuaternion(VectorQuaternion& _data) = 0;

            /// \brief abstract method for receive Velocity
            virtual bool getVelocity(VectorVelocity& _data) = 0;

            /// \brief abstract method for receive Status Flight
            virtual bool getStatusFlight(std::string& _data) = 0;

            /// \brief abstract method for receive Display Mode
            virtual bool getDisplayMode(std::string& _data) = 0;

            /// \brief abstract method for receive Batery Info
            virtual bool getBatery(int& _data) = 0;

            /// \brief abstract method for RC with Flag Data
            virtual bool getRC(VectorRC& _data) = 0;

            /// \brief abstract method for receive Control Device Info
            virtual bool getControlDevice(VectorControlDevice& _data) = 0;

            /// \brief abstract method for receive local position
            virtual bool getLocalPosition(VectorLocalPosition& _data) = 0;

        protected:
            /// \brief abstract method for initialization of the class
            virtual bool init(const Config &_config) = 0;

        protected:
            static Backend *bd_;

    };

	//---------------------------------------------------------------------------------------------------------------------
    class BackendDummy: public Backend{
        public:
            BackendDummy(){}
            ~BackendDummy(){}

            virtual bool emergencyBrake(){return true;}
            virtual bool recoverFromManual(){return true;}
            virtual bool takeOff(const float _height){return true;}
            virtual bool land(){return true;}
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw){return true;}
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate){return true;}
            virtual bool attitudeCtrlVer(float _roll, float _pitch, float _yaw, float _z){return true;}
            virtual bool angularRateCtrlVer(float _rollRate, float _pitchRate, float _yawRate, float _z){return true;}
            virtual bool selfControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP){return true;}
            virtual bool positionCtrlGPS(Eigen::Vector3f _wayPoint, dataMission _config){return true;}
            
            virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config){return true;}
            virtual bool start_mission(){return true;}
            virtual bool pause_mission(){return true;}
            virtual bool stop_mission(){return true;}
            virtual bool resume_mission(){return true;}

            virtual bool initPositionVO(int _freq){return true;}
            virtual bool initGPS(int _freq){return true;}
            virtual bool initGPSDetail(int _freq){return true;}
            virtual bool initGPSSignal(int _freq){return true;}
            virtual bool initAltitude(int _freq){return true;}
            virtual bool initAngularRate(int _freq){return true;}
            virtual bool initHardSync(int _freq){return true;}
            virtual bool initCompass(int _freq){return true;}
            virtual bool initQuaternion(int _freq){return true;}
            virtual bool initVelocity(int _freq){return true;}
            virtual bool initStatusFlight(int _freq){return true;}
            virtual bool initDisplayMode(int _freq){return true;}
            virtual bool initBatery(int _freq){return true;}
            virtual bool initRC(int _freq){return true;}
            virtual bool initControlDevice(int _freq){return true;}
            virtual bool initLocalPosition(){return true;}

            virtual bool getPositionVO(VectorPositionVO& _data){return true;}
            virtual bool getGPS(VectorGPS& _data){return true;}
            virtual bool getGPSDetail(VectorGPSDetail& _data){return true;}
            virtual bool getGPSSignal(int& _data){return true;}
            virtual bool getAltitude(float& _data){return true;}
            virtual bool getAngularRate(VectorAngularRate& _data){return true;}
            virtual bool getHardSync(VectorHardSync& _data){return true;}
            virtual bool getCompass(VectorCompass& _data){return true;}
            virtual bool getQuaternion(VectorQuaternion& _data){return true;}
            virtual bool getVelocity(VectorVelocity& _data){return true;}
            virtual bool getStatusFlight(std::string& _data){return true;}
            virtual bool getDisplayMode(std::string& _data){return true;}
            virtual bool getBatery(int& _data){return true;}
            virtual bool getRC(VectorRC& _data){return true;}
            virtual bool getControlDevice(VectorControlDevice& _data){return true;}
            virtual bool getLocalPosition(VectorLocalPosition& _data){return true;}

    private:
        virtual bool init(const Config &_config){return true;}
    };
}

#endif
