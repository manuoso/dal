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

#include <StateMachine.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC FUNCTIONS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::init(int _argc, char**_argv) {

    if (_argc != 2) {
        std::cout << "No input arguments, using default path" << std::endl;
        path_ = "";
    }else{
        path_ = _argv[1];
    }

    std::ifstream rawFile(path_);
    if (!rawFile.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream;
    strStream << rawFile.rdbuf();
    std::string json = strStream.str();

    if(configFile_.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    int id = configFile_["id"].GetInt();
    std::string key = configFile_["key"].GetString();
    std::string port = configFile_["port"].GetString();
    int baudrate = configFile_["baudrate"].GetInt();
    bool useLogStatus = configFile_["useLogStatus"].GetBool();

    dal::Backend::Config bc;
    bc.app_id = id;
    bc.app_key = key;
    bc.useAdvancedSensing = false;
    bc.device = port;
    bc.baudrate = baudrate;
    bc.type = dal::Backend::Config::eType::DJI;
    bc.useLogStatus = useLogStatus;

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
        return false;
    }

    if(!initFastcom()){
        std::cout << "Error initializing FASTCOM" << std::endl;
        return false;
    }

    telemThread_ = std::thread(&StateMachine::telemetryThread, this);

    state_ = eState::WAIT;
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::run() {

    char msgStatus[20];

    while(fin_ == false){
        switch(state_){
            case eState::WAIT:
            {   
                strcpy(msgStatus, "WAIT");
                break;
            }
            case eState::LAND:
            {   
                strcpy(msgStatus, "LAND");
                break;
            }
            case eState::TAKEOFF:
            {   
                strcpy(msgStatus, "TAKE_OFF");
                break;
            }
            case eState::BRAKE:
            {   
                strcpy(msgStatus, "EMERGENCY_BRAKE");
                break;
            }
            case eState::MOVE_POS:
            {   
                strcpy(msgStatus, "MOVING_POSITION");

                // TODO 666: CONTROL IN POSITION WITH YAW?
                dal::DAL::get()->position(targetPose_.x, targetPose_.y, targetPose_.z, targetPose_.yaw);

                break;
            }
            case eState::MOVE_VEL:
            {   
                strcpy(msgStatus, "MOVING_VELOCITY");

                // TODO 666: CONTROL IN VELOCITY WITH YAW RATE?
                dal::DAL::get()->velocity(targetVelLinear_.x, targetVelLinear_.y, targetVelLinear_.z, targetVelLinear_.yaw);

                break;
            }
            case eState::MOVE_ATT:
            {   
                strcpy(msgStatus, "MOVING_ATT");

                dal::DAL::get()->attitude(targetAtt_.roll, targetAtt_.pitch, targetAtt_.yaw, targetAtt_.z);

                break;
            }
            case eState::MOVE_ATTR:
            {   
                strcpy(msgStatus, "MOVING_ATT_R");

                dal::DAL::get()->attitudeRate(targetAttR_.roll, targetAttR_.pitch, targetAttR_.yaw, targetAttR_.z);

                break;
            }
            case eState::RECOVER_CONTROL:
            {   
                strcpy(msgStatus, "RECOVER_CONTROL");
                break;
            }
            case eState::EXIT:
                std::cout << "\nEXIT..." << std::endl;
                strcpy(msgStatus, "EXIT");
                fin_ = true;
                telemThread_.join();
                break;
        }

        pubState_->publish(msgStatus);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::finalize() {

    fin_ = true;
    telemThread_.join();
    dal::DAL::get()->close();
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE FUNCTIONS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::initFastcom(){

    // Get IP and ports
    std::string ipToGCS = configFile_["ip_to_GCS"].GetString();
    int portState = configFile_["port_state"].GetInt();
    int portPose = configFile_["port_pose"].GetInt();
    int portVel = configFile_["port_vel"].GetInt();
    int portAtt = configFile_["port_att"].GetInt();
    int portAttR = configFile_["port_att_r"].GetInt();
    int portServerHandler = configFile_["port_server_handler"].GetInt();

    int portPubMode = configFile_["port_pub_mode"].GetInt();
    int portPubFS = configFile_["port_pub_fs"].GetInt();
    int portPubRC = configFile_["port_pub_rc"].GetInt();
    int portPubGPS = configFile_["port_pub_gps"].GetInt();
    int portPubVel = configFile_["port_pub_vel"].GetInt();
    int portPubBat = configFile_["port_pub_bat"].GetInt();

    // Initialize Fastcom publishers and subscribers
    pubState_ = new fastcom::Publisher<char[20]>(portState);
    subPose_ = new fastcom::Subscriber<xyz_data>(ipToGCS, portPose);
    subVel_ = new fastcom::Subscriber<xyz_data>(ipToGCS, portVel);
    subAtt_ = new fastcom::Subscriber<rpy_data>(ipToGCS, portAtt);
    subAttR_ = new fastcom::Subscriber<rpy_data>(ipToGCS, portAttR);

    // Initialize Fastcom telemetry publishers
    pubMode_ = new fastcom::Publisher<char[30]>(portPubMode);
    pubFligStatus_ = new fastcom::Publisher<char[30]>(portPubFS);
    pubRC_ = new fastcom::Publisher<rc_data>(portPubRC);
    pubPosGPS_ = new fastcom::Publisher<gps_data>(portPubGPS);
    pubVel_ = new fastcom::Publisher<xyz_data>(portPubVel);
    pubBatLevel_ = new fastcom::Publisher<float>(portPubBat);

    // Callback of received pose
    subPose_->attachCallback([&](const xyz_data &_data){
        targetPose_.x = _data.x;
        targetPose_.y = _data.y;
        targetPose_.z = _data.z;
        targetPose_.yaw = _data.yaw;

        state_ = eState::MOVE_POS;
    });
    
    // Callback of received velocity
    subVel_->attachCallback([&](const xyz_data &_data){
        targetVelLinear_.x = _data.x;
        targetVelLinear_.y = _data.y;
        targetVelLinear_.z = _data.z;
        targetVelLinear_.yaw = _data.yaw;

        state_ = eState::MOVE_VEL;
    });

    // Callback of received velocity
    subAtt_->attachCallback([&](const rpy_data &_data){
        targetAtt_.roll = _data.roll;
        targetAtt_.pitch = _data.pitch;
        targetAtt_.yaw = _data.yaw;
        targetAtt_.z = _data.z;

        state_ = eState::MOVE_ATT;
    });

    // Callback of received velocity
    subAttR_->attachCallback([&](const rpy_data &_data){
        targetAttR_.roll = _data.roll;
        targetAttR_.pitch = _data.pitch;
        targetAttR_.yaw = _data.yaw;
        targetAttR_.z = _data.z;

        state_ = eState::MOVE_ATTR;
    });

    // Callback of service server master
    serverHandler_ = new fastcom::ServiceServer<RequestCommand, ResponseError>(portServerHandler, [&](RequestCommand &_req, ResponseError &_res){
        int rec = _req.command;
        switch(rec){
            case 1: // TAKE OFF
            {   
                state_ = eState::TAKEOFF;
                dal::DAL::get()->takeOff(1);

                state_ = eState::WAIT;
                _res.error = 1;
                break;
            }
            case 2: // LAND
            {   
                state_ = eState::LAND;
                dal::DAL::get()->land();

                state_ = eState::WAIT;
                _res.error = 1;
                break;
            }
            case 3: // EMERGENCY BRAKE
            {   
                state_ = eState::BRAKE;
                dal::DAL::get()->emergencyBrake();

                targetVelLinear_.x = 0;
                targetVelLinear_.y = 0;
                targetVelLinear_.z = 0;
                targetVelLinear_.yaw = 0;

                targetPose_.x = 0;
                targetPose_.y = 0;
                targetPose_.z = 0;
                targetPose_.yaw = 0;

                state_ = eState::WAIT;
                _res.error = 1;
                break;
            }

        }

    });

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::telemetryThread() {

    std::cout << "Start Telemetry Thread" << std::endl;

    while (fin_ == false) { 

        Eigen::Vector2f gps;
        if(!dal::DAL::get()->telemetryGPS(gps)){
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

        char msgFS[30];
        strcpy(msgFS, statusFlight.c_str());

        char msgMode[30];
        strcpy(msgMode, mode.c_str());

        rc_data msgRC;
        msgRC.rc1 = rc(0);
        msgRC.rc2 = rc(1);
        msgRC.rc3 = rc(2);
        msgRC.rc4 = rc(3);
        msgRC.flag1 = rc(4);
        msgRC.flag2 = rc(5);
        msgRC.flag3 = rc(6);
        msgRC.flag4 = rc(7);

        gps_data msgGPS;
        msgGPS.latitude = RAD_DEG(gps(0));
        msgGPS.longitude = RAD_DEG(gps(1));
        msgGPS.altitude = altitude;

        xyz_data msgVel;
        msgVel.x = vel(0);
        msgVel.y = vel(1);
        msgVel.z = vel(2);

        float batLevel = bat/1000.0;

        pubMode_->publish(msgMode);
        pubFligStatus_->publish(msgFS);
        pubRC_->publish(msgRC);
        pubPosGPS_->publish(msgGPS);
        pubVel_->publish(msgVel);
        pubBatLevel_->publish(batLevel);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
    }
    
    return true;
}
