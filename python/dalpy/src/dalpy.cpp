
#include <boost/python.hpp>
#include <boost/python/dict.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <fstream>

#include <Eigen/Eigen>

#include <dal/dal.h>

#define CONST_PI (double)3.141592653589793
#define DEG_RAD(DEG) ((DEG) * ((CONST_PI) / (180.0)))
#define RAD_DEG(RAD) ((RAD) * (180.0) / (CONST_PI))

using namespace boost::python;

class PAL
{
    public:
        //----------------------------------------------------------------------------------------------------
        PAL(int id, std::string key, std::string port, bool log){
            id_ = id;
            key_ = key;
            port_ = port;
            useLog_ = log;
            configure_DAL();
        }

        //----------------------------------------------------------------------------------------------------
        void exit(){
            Py_Finalize();
            dal::DAL::close();    
        }

        //----------------------------------------------------------------------------------------------------
        bool takeoff(){ 
            return djiManager_->control()->takeOff(1);
            
        }

        //----------------------------------------------------------------------------------------------------
        bool land(){ 
            return djiManager_->control()->land();
        }

        //----------------------------------------------------------------------------------------------------
        bool emergencyBrake(){ 
            return djiManager_->control()->emergencyBrake(); 
        }

        //----------------------------------------------------------------------------------------------------
        bool recoverFromManual(){ 
            return djiManager_->control()->recoverFromManual();
        }

        //----------------------------------------------------------------------------------------------------
        bool position(float x, float y, float z, float yaw){ 
            return djiManager_->control()->position(x, y, z, yaw);
        }

        //----------------------------------------------------------------------------------------------------
        bool velocity(float vx, float vy, float vz, float yawRate){ 
            return djiManager_->control()->velocity(vx, vy, vz, yawRate);
        }

        //----------------------------------------------------------------------------------------------------
        bool attitude(float roll, float pitch, float yaw, float z){ 
            return djiManager_->control()->attitude(roll, pitch, yaw, z);
        }

        //----------------------------------------------------------------------------------------------------
        bool attitudeRate(float rollRate, float pitchRate, float yawRate, float z){ 
            return djiManager_->control()->attitudeRate(rollRate, pitchRate, yawRate, z);
        }

        //----------------------------------------------------------------------------------------------------
        bool rpyThrust(float roll, float pitch, float yawRate, float thrust){ 
            return djiManager_->control()->rpyThrust(roll, pitch, yawRate, thrust);
        }

        //----------------------------------------------------------------------------------------------------
        boost::python::dict telemetry(){ 
            dal::TelemetryDJI::VectorGPS gps;
            if(!djiManager_->telemetry()->getGPS(gps)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorGPSDetail gpsDetail;
            if(!djiManager_->telemetry()->getGPSDetail(gpsDetail)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            int gpsSignal;
            if(!djiManager_->telemetry()->getGPSSignal(gpsSignal)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            float altitude;
            if(!djiManager_->telemetry()->getAltitude(altitude)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorAngularRate angularRate;
            if(!djiManager_->telemetry()->getAngularRate(angularRate)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorHardSync imu;
            if(!djiManager_->telemetry()->getHardSync(imu)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorCompass compass;
            if(!djiManager_->telemetry()->getCompass(compass)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorQuaternion quat;
            if(!djiManager_->telemetry()->getQuaternion(quat)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorVelocity vel;
            if(!djiManager_->telemetry()->getVelocity(vel)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            std::string statusFlight;
            if(!djiManager_->telemetry()->getStatusFlight(statusFlight)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            std::string mode;
            if(!djiManager_->telemetry()->getDisplayMode(mode)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            int bat;
            if(!djiManager_->telemetry()->getBatery(bat)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::TelemetryDJI::VectorRC rc;
            if(!djiManager_->telemetry()->getRC(rc)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            Eigen::Vector3f localPositionGPS;
            if(!djiManager_->telemetry()->getLocalPositionGPS(localPositionGPS)){
                std::cout << "Error getting localPositionGPS of A3" << std::endl;
            }

            boost::python::dict telem;
            telem["Flight_Status"] = statusFlight.c_str();
            telem["Mode"] = mode.c_str();
            telem["Batery_Level"] = bat/1000.0;
            
            telem["Pitch"] = rc(0);
            telem["Roll"] = rc(1);
            telem["Yaw"] = rc(2);
            telem["Throttle"] = rc(3);

            telem["GPS_Latitude"] = RAD_DEG(gps[0]);
            telem["GPS_Longitude"] = RAD_DEG(gps[1]);
            telem["GPS_Altitude"] = altitude;
            telem["GPS_Signal"] = gpsSignal;

            telem["Velocity_X"] = vel[0];
            telem["Velocity_Y"] = vel[1];
            telem["Velocity_Z"] = vel[2];

            telem["AngularRate_X"] = angularRate[0];
            telem["AngularRate_Y"] = angularRate[1];
            telem["AngularRate_Z"] = angularRate[2];

            telem["compass_X"] = compass[0];
            telem["compass_Y"] = compass[1];
            telem["compass_Z"] = compass[2];

            telem["LocalPoseGPS_X"] = localPositionGPS[0];
            telem["LocalPoseGPS_Y"] = localPositionGPS[1];
            telem["LocalPoseGPS_Z"] = localPositionGPS[2];

            // 666 TODO: CHECK IMU ORI INFO!!
            telem["IMU_angular_velocity_x"] = imu[0];
            telem["IMU_angular_velocity_y"] = imu[1];
            telem["IMU_angular_velocity_z"] = imu[2];

            telem["IMU_linear_acceleration_x"] = imu[3];
            telem["IMU_linear_acceleration_y"] = imu[4];
            telem["IMU_linear_acceleration_z"] = imu[5];

            telem["IMU_orientation_w"] = imu[6];
            telem["IMU_orientation_x"] = imu[7];
            telem["IMU_orientation_y"] = imu[8];
            telem["IMU_orientation_z"] = imu[9];

            // 666 TODO: CHECK CONVERSIONS!!
            telem["quaternion_w"] = quat[0];
            telem["quaternion_x"] = quat[1];
            telem["quaternion_y"] = quat[2];
            telem["quaternion_z"] = quat[3];

            return telem;

        }

    private:
        //----------------------------------------------------------------------------------------------------
        void configure_DAL(){
            dal::HAL::Config bc;
            bc.app_id = id_;
            bc.app_key = key_;
            bc.useAdvancedSensing = false;
            bc.device = port_;
            bc.baudrate = baudrate_;
            bc.useLog = useLog_;

            std::map<DJI::OSDK::Telemetry::TopicName, int> topics;
            bc.topics = topics;

            djiManager_ = dal::DAL::create(bc);

            if(djiManager_ == nullptr){
                std::cout << "Error initializing DAL" << std::endl;
            }
        }

    private:
        dal::DAL *djiManager_ = nullptr;

        int id_, baudrate_;
        std::string key_, port_;
        bool useLog_;
    
};

BOOST_PYTHON_MODULE(dalpy)
{
    class_<PAL>("PAL", init<int, std::string, std::string, bool>())
        .def("__del__", &PAL::exit) 
        .def("takeoff", &PAL::takeoff)
        .def("land", &PAL::land)
        .def("emergencyBrake", &PAL::emergencyBrake)
        .def("recoverFromManual", &PAL::recoverFromManual)
        .def("position", &PAL::position)
        .def("velocity", &PAL::velocity)
        .def("attitude", &PAL::attitude)
        .def("attitudeRate", &PAL::attitudeRate)
        .def("rpyThrust", &PAL::rpyThrust)        
        .def("telemetry", &PAL::telemetry)

    ;
};
