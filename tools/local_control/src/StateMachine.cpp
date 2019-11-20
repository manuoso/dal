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
        path_ = "/home/aphrodite/programming/dal/tools/local_control/config/config_dal.json";
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
    topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_VELOCITY, 200));
    topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT, 50));
    topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE, 50));
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

    posThread_ = std::thread(&StateMachine::poseThread, this);

    plotThread_ = std::thread(&StateMachine::plotThread, this);

    state_ = eState::WAIT;

    lastTime_ = std::chrono::steady_clock::now();
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::run() {

    int action;

    while(fin_ == false){
        switch(state_){
            case eState::WAIT:
            {   
                std::cout << "Select action: " << std::endl;
                std::cout << "- 1: Take off" << std::endl;
                std::cout << "- 2: Land" << std::endl;
                std::cout << "- 3: Emergency Brake" << std::endl;
                std::cout << "- 4: Local Control" << std::endl;
                std::cout << "- 0: Exit" << std::endl;
                std::cin >> action;
                if(action == 1){
                    state_ = eState::TAKEOFF;
                }else if(action == 2){ 
                    state_ = eState::LAND;
                }else if(action == 3){ 
                    state_ = eState::BRAKE;
                }else if(action == 4){                 
                    state_ = eState::LOCAL_CONTROL;
                }else if(action == 0){                 
                    state_ = eState::EXIT;             
                }else{
                    std::cout << "Unrecognized action, try again." << std::endl;
                }
                break;
            }
            case eState::LAND:
            {   
                dal::DAL::get()->land();
                state_ = eState::WAIT;
                break;
            }
            case eState::TAKEOFF:
            {   
                dal::DAL::get()->takeOff(1);
                state_ = eState::WAIT;
                break;
            }
            case eState::BRAKE:
            {   
                dal::DAL::get()->emergencyBrake();
                state_ = eState::WAIT;
                break;
            }
            case eState::LOCAL_CONTROL:
            {   
                float x, y, z;
                std::cout << "Enter ref: " << std::endl;
                std::cout << "X: ";
                std::cin >> x;
                std::cout << "Y: ";
                std::cin >> y;
                std::cout << "Z: ";
                std::cin >> z;

                dal::DAL::get()->setReference(x, y, z);
                localControlThread_ = std::thread(&StateMachine::controlThread, this);

                state_ = eState::CHANGE_REF;

                break;
            }
            case eState::CHANGE_REF:
            {   
                std::cout << "Want to change ref? 1: YES | 2: FINISH CONTROL | 0: EXIT" << std::endl;
                std::cin >> action;
                if(action == 1){
                    float x, y, z;
                    std::cout << "Enter ref: " << std::endl;
                    std::cout << "X: ";
                    std::cin >> x;
                    std::cout << "Y: ";
                    std::cin >> y;
                    std::cout << "Z: ";
                    std::cin >> z;

                    dal::DAL::get()->setReference(x, y, z);
                }else if(action == 2){ 
                    finControl_ = true;    
                    localControlThread_.join();
                    state_ = eState::WAIT;  
                }else if(action == 0){ 
                    state_ = eState::EXIT;           
                }else{
                    std::cout << "Unrecognized action, try again." << std::endl;
                }

                break;
            }
            case eState::EXIT:
                std::cout << "\nEXIT..." << std::endl;
                fin_ = true;
                telemThread_.join();
                posThread_.join();
                break;
        }

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

    int portPubMode = configFile_["port_pub_mode"].GetInt();
    int portPubFS = configFile_["port_pub_fs"].GetInt();
    int portPubRC = configFile_["port_pub_rc"].GetInt();
    int portPubVel = configFile_["port_pub_vel"].GetInt();

    // Initialize Fastcom telemetry publishers
    pubMode_ = new fastcom::Publisher<char[30]>(portPubMode);
    pubFligStatus_ = new fastcom::Publisher<char[30]>(portPubFS);
    pubRC_ = new fastcom::Publisher<rc_data>(portPubRC);
    pubVel_ = new fastcom::Publisher<xyz_data>(portPubVel);

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::telemetryThread() {

    std::cout << "Start Telemetry Thread" << std::endl;

    while (fin_ == false) { 

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

        xyz_data msgVel;
        msgVel.x = vel(0);
        msgVel.y = vel(1);
        msgVel.z = vel(2);

        pubMode_->publish(msgMode);
        pubFligStatus_->publish(msgFS);
        pubRC_->publish(msgRC);
        pubVel_->publish(msgVel);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
    }
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::poseThread(){

    std::cout << "Start Pose Thread" << std::endl;

    while (fin_ == false) { 
        
        poseLocalControl_.x = 0.0;
        poseLocalControl_.y = 0.0;
        poseLocalControl_.z = 0.0;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
    }

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::controlThread(){

    std::cout << "Start Control Thread" << std::endl;

    while (finControl_ == false) { 
        
        Eigen::Vector3f vel;
        secureGuard_.lock();
        vel = dal::DAL::get()->localControl(poseLocalControl_.x, poseLocalControl_.y, poseLocalControl_.z);
        secureGuard_.unlock();

        targetVelLinear_.x = vel(0);
        targetVelLinear_.y = vel(1);
        targetVelLinear_.z = vel(2);
        targetVelLinear_.yaw = 0.0;

        dal::DAL::get()->velocity(targetVelLinear_.x, targetVelLinear_.y, targetVelLinear_.z, targetVelLinear_.yaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
    }

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::plotThread(){

    std::cout << "Start Plot Thread" << std::endl;

    std::vector<float> t, x, y, z;

    while (fin_ == false) { 

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        auto key = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTime_).count();
        if(key > 5){
            t.push_back(key);

            x.push_back(poseLocalControl_.x);
            y.push_back(poseLocalControl_.y);
            z.push_back(poseLocalControl_.z);

            

            lastTime_ = t1;

        }    
        
    }

    return true;
}
