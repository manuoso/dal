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


#ifndef DAL_BACKENDS_BACKEND_H_
#define DAL_BACKENDS_BACKEND_H_

#include <string>
#include <Eigen/Eigen>

namespace dal{
    class Backend{
        public:
            struct Config{
                /// Type of backend
                enum class eType {DJI, APM, PX4, Dummy};
                eType type;

                /// ID of DJI APP
                int app_id;
                
                /// KEY of DJI APP
                std::string app_key = "";

                /// Use or not advance sensing for DJI
                bool useAdvancedSensing = false;

                /// Device port of the controller
                std::string device = "";

                /// Baudrate to connect to the controller
                unsigned int baudrate;

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

            /// Structs that we use to receive Telemetry
            struct dataTelemetry{
                std::string         flightStatus;
                std::string         mode;
                Eigen::Vector2f     latLon;
                double              altitude;
                int                 nGPS; 
                int                 batteryVoltage;
                Eigen::VectorXf     imu = Eigen::VectorXf(10);
                Eigen::Vector3f     localPosition;
                Eigen::Vector4f     rc;
                Eigen::Vector3f     velocity;
                Eigen::Vector4f     quaternion;
                Eigen::VectorXf     rtk = Eigen::VectorXf(9);
            };

            static Backend* create(const Config &_config);

            /// \brief abstract method for take off
            virtual bool takeOff(const float _height) = 0;

            /// \brief abstract method for land
            virtual bool land() = 0;

            /// \brief abstract method for emergency brake
            virtual bool emergencyBrake() = 0;

            /// \brief abstract method for recover control
            virtual bool recoverFromManual() = 0;

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
	
            /// \brief abstract method for position control and yaw
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw) = 0;
	    
            /// \brief abstract method for velocity control and yaw
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate) = 0;
            
            /// \brief abstract method for receive telemetry of the uav
            virtual bool receiveTelemetry(dataTelemetry& _data, bool _saveToFile) = 0;

        protected:
            Backend() {}  
            /// \brief abstract method for initialization of the class
            virtual bool init(const Config &_config) = 0;

    };

    class BackendDummy: public Backend{
        virtual bool takeOff(const float _height){return true;}
        virtual bool land(){return true;}
        virtual bool emergencyBrake(){return true;}
        virtual bool recoverFromManual(){return true;}
        virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config){return true;}
        virtual bool start_mission(){return true;}
        virtual bool pause_mission(){return true;}
        virtual bool stop_mission(){return true;}
        virtual bool resume_mission(){return true;}
        virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw){return true;}
        virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate){return true;}
        virtual bool receiveTelemetry(dataTelemetry& _data, bool _saveToFile){return true;}

    private:
        virtual bool init(const Config &_config){return true;}
    };
}

#endif
