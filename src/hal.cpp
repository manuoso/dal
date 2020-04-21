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


#include <dal/hal.h>

namespace dal{
    DJI::OSDK::Vehicle *HAL::vehicle_ = nullptr;

    int HAL::functionTimeout_ = 1;

    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type HAL::originAltitude_ = 0;
    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type HAL::originGPS_ = {0, 0, 0, 0};

    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    HAL::HAL() {}

    //---------------------------------------------------------------------------------------------------------------------
    HAL::~HAL() {}

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::create(const Config &_config){

        std::cout <<    "\033[34m     ........               ........    \n"
                        "  .............           ............. \n"
                        " ...............         ...............\n"
                        " .......▄........       ........▄.......\n"
                        " .......▄▄▄......       ......▄▄▄.......\n"
                        " .........▄▄▄...         ...▄▄▄.........\n"
                        "  ..........▄▄▄  ▄▄▄▄▄▄▄  ▄▄▄.......... \n"
                        "    ......... ▄▄▄▄▄▄▄▄▄▄▄▄▄ .........   \n"
                        "	        ▄▄▄▄▄▄▄▄▄               \n"
                        "		 ▄▄▄▄▄▄▄                \n"
                        "		 ▄▄▄▄▄▄▄                \n"
                        "	        ▄▄▄▄▄▄▄▄▄               \n"
                        "    ......... ▄▄▄▄▄▄▄▄▄▄▄▄▄ .........   \n"
                        "  ..........▄▄▄  ▄▄▄▄▄▄▄  ▄▄▄.......... \n"
                        " .........▄▄▄...         ...▄▄▄.........\n"
                        " .......▄▄▄......       ......▄▄▄.......\n"
                        " .......▄........       ........▄.......\n"
                        " ...............         ...............\n"
                        "  .............           ............. \n"
                        "  ____________________________________  \n"
                        " |                                    | \n"
                        " | Initializing DJI backend - Manuoso | \n"
                        " |____________________________________| \033[m\n";

        if(_config.useLogDJI){
            // DJI::OSDK::Log::instance().enableDebugLogging();
            DJI::OSDK::Log::instance().enableStatusLogging();
        }else{
            // DJI::OSDK::Log::instance().disableDebugLogging();
            DJI::OSDK::Log::instance().disableStatusLogging();
        }        
        
        // Setup Vehicle
        vehicle_ = new DJI::OSDK::Vehicle(_config.device.c_str(), _config.baudrate, true, _config.useAdvancedSensing);

        if(vehicle_ == nullptr){
            std::cout << "\033[31mVehicle not initialized, exiting \033[m" << std::endl;
            return false;
        }

        // Check if the communication is working fine
        if(!vehicle_->protocolLayer->getDriver()->getDeviceStatus()){
            std::cout << "\033[31mComms appear to be incorrectly set up exiting \033[m" << std::endl;
            return false;
        }

        // Activate
        DJI::OSDK::Vehicle::ActivateData activateData;
        activateData.ID = _config.app_id;
        char app_key[65];
        activateData.encKey = app_key;
        std::strcpy(activateData.encKey, _config.app_key.c_str());

        activateData.version = vehicle_->getFwVersion();
        DJI::OSDK::ACK::ErrorCode ack = vehicle_->activate(&activateData, functionTimeout_);

        if(DJI::OSDK::ACK::getError(ack)){
            DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
            std::cout << "\033[31mError validating Vehicle, exiting \033[m" << std::endl;
            return false;
        }

        std::cout << "\033[32mVehicle initialized \033[m" << std::endl;

        // Obtain Control Authority
        if(obtainControlAuthority()){
            if(_config.topics.size() > 0){
                std::cout << "\033[32mUsing std map \033[m" << std::endl;
                if(extractTopics(_config.topics)){
                    // Wait for the data to start coming in.
                    sleep(3);

                    return setLocalPosition();
                }else{
                    return false;
                }                
            }else{
                std::cout << "\033[32mUsing default topics \033[m" << std::endl;

                std::map<DJI::OSDK::Telemetry::TopicName, int> topicsDefault;
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS, 1));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_HARD_SYNC, 200));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED, 200));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_QUATERNION, 200));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_VELOCITY, 200));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_GPS_FUSED, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_COMPASS, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA, 50));
                topicsDefault.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA, 50));
                // topics.insert(std::make_pair(DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE, 50));

                if(extractTopics(topicsDefault)){
                    // Wait for the data to start coming in.
                    sleep(3);

                    return setLocalPosition();
                }else{
                    return false;
                } 
            }
        }else{
            return false;
        }    
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    void HAL::close(){
        unsubscribeAllTopics();

        delete vehicle_;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR CHECKS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::verify(){

        DJI::OSDK::ACK::ErrorCode subscribeStatus = vehicle_->subscribe->verify(functionTimeout_);
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            std::cout << "\033[31mNot verified subscription to Telemetry, exiting \033[m" << std::endl;
            return false;
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // UTILS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::obtainControlAuthority(){ 

        DJI::OSDK::ACK::ErrorCode ctrlStatus = HAL::vehicle_->obtainCtrlAuthority(HAL::functionTimeout_);
        if (DJI::OSDK::ACK::getError(ctrlStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(ctrlStatus, __func__);
            std::cout << "\033[31mNot obtained Control Authority, exiting \033[m" << std::endl;
            return false;
        }
        std::cout << "\033[32mObtained Control Authority of the Vehicle \033[m" << std::endl;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::extractTopics(std::map<DJI::OSDK::Telemetry::TopicName, int> _topics){
        bool result;
        int searchFreq = 0, contMaxFreq = 0;
        std::map<DJI::OSDK::Telemetry::TopicName, int>::iterator it, itSameFreq;
        std::map<DJI::OSDK::Telemetry::TopicName, int> copyConfigTopics = _topics;
        it = copyConfigTopics.begin();

        while (copyConfigTopics.size() != 0) {
            searchFreq = it->second;

            contMaxFreq++;
            if (contMaxFreq > 4) {
                std::cout << "\033[31mUsing more packages with different frequencies than DJI allows, exiting \033[m" << std::endl;
                return 0;
            }

            itSameFreq = copyConfigTopics.begin();
            std::vector<DJI::OSDK::Telemetry::TopicName> topics;
            while (itSameFreq != copyConfigTopics.end()) {
                if (searchFreq == itSameFreq->second) {
                    topics.push_back(itSameFreq->first);
                    itSameFreq = copyConfigTopics.erase(itSameFreq);
                }else {
                    itSameFreq++;
                }
            }

            if (topics.size() > 0) {
                result = subscribeToTopic(topics, searchFreq);
                if(!result){
                    return false;
                }
            }

            it = copyConfigTopics.begin();
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::setLocalPosition(){

        originGPS_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        originAltitude_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
        
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::subscribeToTopic(std::vector<DJI::OSDK::Telemetry::TopicName> _topics, int _freq){

        DJI::OSDK::ACK::ErrorCode subscribeStatus = vehicle_->subscribe->verify(functionTimeout_);
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            std::cout << "\033[31mNot verified subscription to Telemetry, exiting \033[m" << std::endl;
            return false;
        }

        DJI::OSDK::Telemetry::TopicName topicList[_topics.size()];   
        for(unsigned int i = 0; i < _topics.size(); i++ ){
            topicList[i] = _topics[i];
        }

        int numTopic = sizeof(topicList) / sizeof(topicList[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList, enableTimestamp, _freq);
        if (!pkgStatus){
            std::cout << "\033[31mNot init package 0 from topic list, exiting \033[m" << std::endl;
            return false;
        }

        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeAllTopics(); 
            std::cout << "\033[31mStart package 0 error, exiting \033[m" << std::endl;
            return false;
        }

        pkgIndex_++;
        
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool HAL::unsubscribeAllTopics(){

        for(int i = 0; i < pkgIndex_; i++ ){
            DJI::OSDK::ACK::ErrorCode ack = vehicle_->subscribe->removePackage(i, functionTimeout_);
            if (DJI::OSDK::ACK::getError(ack)){
                std::cout << "\033[31mError unsubscribing at package: \033[m" + std::to_string(i) + "\033[31mplease restart the drone/FC to get back to a clean state, exiting \033[m" << std::endl;
                return false;
            }
        }
        
        return true;
    }

}
