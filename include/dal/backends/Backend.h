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

            /// Structs that we use to receive Telemetry
            struct dataTelemetry{
                std::string flightStatus;
                std::string mode;
                Eigen::Vector2f latLon;
                int nGPS; 
                Eigen::Vector3f localPositionNED;
                Eigen::Vector3f localPositionENU;
                double altitude;
                Eigen::Vector4f rc;
                Eigen::Vector3f velocity;
                Eigen::Vector4f quaternion;
                Eigen::VectorXf rtk = Eigen::VectorXf(9);
            };

            static Backend* create(const Config &_config);

            /// \brief abstract method for take off
            virtual bool takeOff(const float _height) = 0;

            /// \brief abstract method for land
            virtual bool land() = 0;

            /// \brief abstract method for emergency brake
            virtual bool emergencyBrake() = 0;

            /// \brief abstract method for configure a desired mission given the waypoints
            virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints) = 0;

            /// \brief abstract method for start a configured mission
            virtual bool start_mission() = 0;
	
            /// \brief abstract method for position control and yaw using DJI SDK.
            virtual bool movePosition(float _x, float _y, float _z, float _yaw, float _posThreshold = 0.2, float _yawThreshold = 1.0) = 0;

            /// \brief abstract method for position control and yaw
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw, bool _offset) = 0;
	    
            /// \brief abstract method for velocity control and yaw
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate) = 0;
            
            /// \brief abstract method for receive telemetry of the uav
            virtual bool receiveTelemetry(dataTelemetry& _data, bool _printData, bool _saveToFile) = 0;

        protected:
            Backend() {}  
            /// \brief abstract method for initialization of the class
            virtual bool init(const Config &_config) = 0;

    };

    class BackendDummy: public Backend{
        virtual bool takeOff(const float _height){return true;}
        virtual bool land(){return true;}
        virtual bool emergencyBrake(){return true;}
        virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints){return true;}
        virtual bool start_mission(){return true;}
        virtual bool movePosition(float _x, float _y, float _z, float _yaw, float _posThreshold = 0.2, float _yawThreshold = 1.0){return true;}
        virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw, bool _offset){return true;}
        virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate){return true;}
        virtual bool receiveTelemetry(dataTelemetry& _data, bool _printData, bool _saveToFile){return true;}

    private:
        virtual bool init(const Config &_config){return true;}
    };
}

#endif
