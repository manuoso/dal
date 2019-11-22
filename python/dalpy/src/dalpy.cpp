
#include <boost/python.hpp>
#include <boost/python/dict.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <fstream>

#ifdef HAS_RJSON
    #include <rapidjson/document.h>
#endif

#include <dal/dal.h>


#define CONST_PI (double)3.141592653589793
#define DEG_RAD(DEG) ((DEG) * ((CONST_PI) / (180.0)))
#define RAD_DEG(RAD) ((RAD) * (180.0) / (CONST_PI))

using namespace boost::python;

class PAL
{
    public:
        #ifdef HAS_RJSON
            //----------------------------------------------------------------------------------------------------
            PAL(std::string config){ 
                Py_Initialize();
                if(config == "dummy"){
                    configure_DAL(true);
                }else{
                    configure_json(config); 
                    configure_DAL(false);
                }  
            }
        #else
            //----------------------------------------------------------------------------------------------------
            PAL(std::string config){ 
                std::cout << "Not found RapidJSON, please use: PAL(int id, std::string key, std::string port, bool PosVO, bool LogSta, bool LogTel, bool dummy)" << std::endl;
            }
        #endif

        //----------------------------------------------------------------------------------------------------
        PAL(int id, std::string key, std::string port, bool LogSta, bool dummy){
            id_ = id;
            key_ = key;
            port_ = port;
            useLogStatus_ = LogSta;
            configure_DAL(dummy);
        }

        //----------------------------------------------------------------------------------------------------
        void exit(){
            Py_Finalize();
            dal::DAL::get()->close();    
        }

        //----------------------------------------------------------------------------------------------------
        bool takeoff(){ 
            return dal::DAL::get()->takeOff(1);
        }

        //----------------------------------------------------------------------------------------------------
        bool land(){ 
            return dal::DAL::get()->land();
        }

        //----------------------------------------------------------------------------------------------------
        bool emergencyBrake(){ 
            return dal::DAL::get()->emergencyBrake(); 
        }

        //----------------------------------------------------------------------------------------------------
        bool recoverFromManual(){ 
            return dal::DAL::get()->recoverFromManual();
        }

        //----------------------------------------------------------------------------------------------------
        bool position(float x, float y, float z, float yaw){ 
            return dal::DAL::get()->position(x, y, z, yaw);
        }

        //----------------------------------------------------------------------------------------------------
        bool velocity(float vx, float vy, float vz, float yawRate){ 
            return dal::DAL::get()->velocity(vx, vy, vz, yawRate);
        }

        //----------------------------------------------------------------------------------------------------
        bool attitude(float roll, float pitch, float yaw, float z){ 
            return dal::DAL::get()->attitude(roll, pitch, yaw, z);
        }

        //----------------------------------------------------------------------------------------------------
        bool attitudeRate(float rollRate, float pitchRate, float yawRate, float z){ 
            return dal::DAL::get()->attitudeRate(rollRate, pitchRate, yawRate, z);
        }

        //----------------------------------------------------------------------------------------------------
        boost::python::dict telemetry(){ 
            Eigen::Vector2f gps;
            if(!dal::DAL::get()->telemetryGPS(gps)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            int gpsSignal;
            if(!dal::DAL::get()->telemetryGPSSignal(gpsSignal)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            float altitude;
            if(!dal::DAL::get()->telemetryAltitude(altitude)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            Eigen::Vector3f vel;
            if(!dal::DAL::get()->telemetryVelocity(vel)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            std::string statusFlight;
            if(!dal::DAL::get()->telemetryStatusFlight(statusFlight)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            std::string mode;
            if(!dal::DAL::get()->telemetryDisplayMode(mode)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            int bat;
            if(!dal::DAL::get()->telemetryBatery(bat)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }

            dal::Backend::Backend::VectorRC rc;
            if(!dal::DAL::get()->telemetryRC(rc)){
                std::cout << "Error getting telemetry of A3" << std::endl;
            }
            
            boost::python::dict telem;
            telem["Flight_Status"] = statusFlight.c_str();
            telem["Mode"] = mode.c_str();

            telem["Pitch"] = rc(0);
            telem["Roll"] = rc(1);
            telem["Yaw"] = rc(2);
            telem["Throttle"] = rc(3);
            telem["LogicConnected"] = rc(4);
            telem["SkyConnected"] = rc(5);
            telem["GroundConnected"] = rc(6);
            telem["AppConnected"] = rc(7);

            telem["GPS_Latitude"] = RAD_DEG(gps(0));
            telem["GPS_Longitude"] = RAD_DEG(gps(1));
            telem["GPS_Altitude"] = altitude;
            telem["GPS_Signal"] = gpsSignal;

            telem["Velocity_X"] = vel(0);
            telem["Velocity_Y"] = vel(1);
            telem["Velocity_Z"] = vel(2);

            telem["Batery_Level"] = bat/1000.0;

            return telem;

        }

    private:
        #ifdef HAS_RJSON
            //----------------------------------------------------------------------------------------------------
            void configure_json(std::string _config){
                std::ifstream rawFile(_config);
                if (!rawFile.is_open()) {
                    std::cout << "Error opening config file" << std::endl;
                }

                std::stringstream strStream;
                strStream << rawFile.rdbuf();
                std::string json = strStream.str();

                rapidjson::Document configFile;
                if(configFile.Parse(json.c_str()).HasParseError()){
                    std::cout << "Error parsing json" << std::endl;
                }

                id_ = configFile["id"].GetInt();
                key_ = configFile["key"].GetString();
                port_ = configFile["port"].GetString();
                baudrate_ = configFile["baudrate"].GetInt();
                useLogStatus_ = configFile["useLogStatus"].GetBool();

            }
        #endif

        //----------------------------------------------------------------------------------------------------
        void configure_DAL(bool _dummy){
            dal::Backend::Config bc;
            bc.app_id = id_;
            bc.app_key = key_;
            bc.useAdvancedSensing = false;
            bc.device = port_;
            bc.baudrate = baudrate_;
            if(_dummy){
                bc.type = dal::Backend::Config::eType::Dummy;
            }else{
                bc.type = dal::Backend::Config::eType::DJI;
            }
            bc.useLogStatus = useLogStatus_;

            std::map<DJI::OSDK::Telemetry::TopicName, int> topics;
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_GPS_FUSED, 50));
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED, 50));
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_VELOCITY, 200));
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT, 50));
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE, 50));
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO, 50));
            topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA, 50));

            bc.topics = topics;

            dal::DAL::init(bc);

            if(dal::DAL::get()->backend() == nullptr){
                std::cout << "Error initializing DAL" << std::endl;
            }

        }

    private:
        int id_, baudrate_;
        std::string key_, port_;
        bool useLogStatus_;
    
};

BOOST_PYTHON_MODULE(dalpy)
{
    class_<PAL>("PAL", init<std::string>())
        .def(init<int, std::string, std::string, bool, bool>())
        .def("__del__", &PAL::exit) 
        .def("takeoff", &PAL::takeoff)
        .def("land", &PAL::land)
        .def("emergencyBrake", &PAL::emergencyBrake)
        .def("recoverFromManual", &PAL::recoverFromManual)
        .def("position", &PAL::position)
        .def("velocity", &PAL::velocity)
        .def("attitude", &PAL::attitude)
        .def("attitudeRate", &PAL::attitudeRate)
        .def("telemetry", &PAL::telemetry)

    ;
};
