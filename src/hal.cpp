//---------------------------------------------------------------------------------------------------------------------
//  DJI ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2021 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
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

#include <iostream> // TODO: DELETE THIS

#include <dal/hal.hpp>
#include <dal/parsers/tinyxml2/tinyxml2.h>

namespace dal {

    using namespace DJI::OSDK;
    using namespace DJI::OSDK::Telemetry;
    
    // ----------------------------------------------------------------------
    HAL::HAL() 
        : started_(false)
        , pkgIndex_(0)
        , vehicle_(nullptr)
        , functionTimeout_(1)
    {
        originAltitude_ = 0;
        originGPS_ = {0, 0, 0, 0};
        // TODO: SETUP LOGGER AND DELETE COUTs?
    }

    // ----------------------------------------------------------------------
    HAL::~HAL()
    {
        this->stop();
    }

    // ----------------------------------------------------------------------
    void HAL::stop()
    {
        if (started_)
        {
            started_ = false;
            unsubscribeAllTopics();
            delete vehicle_;
        }
    }

    // ----------------------------------------------------------------------
    bool HAL::produceMinimal()
    {
        if (started_)
            return false;

        showLogo();
        if (!extract(cfg_))
            return false;
        
        showCfg(cfg_);
        if (!init(cfg_))
            return false;

        if (!topicsMinimal())
            return false;

        started_ = true;

        return true;
    }

    // ----------------------------------------------------------------------
    bool HAL::produceAll()
    {
        if (started_)
            return false;

        showLogo();
        if (!extract(cfg_))
            return false;
        
        showCfg(cfg_);
        if (!init(cfg_))
            return false;

        if (!topicsAll())
            return false;

        started_ = true;

        return true;
    }

    // ----------------------------------------------------------------------
    bool HAL::produceCustom(Config _cfg, Topics _topics)
    {
        if (started_)
            return false;

        cfg_ = _cfg;
        showLogo();
        showCfg(cfg_);
        if (!init(cfg_))
            return false;

        if (!topicsCustom(_topics))
            return false;

        started_ = true;

        return true;
    }


    // ----------------------------------------------------------------------
    Vehicle* HAL::getVehicle()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!started_)
            return nullptr;
        else
            return vehicle_;
    }

    // ----------------------------------------------------------------------
    bool HAL::init(Config _cfg)
    {   
        if (_cfg.useLogDJI)
        {
            // Log::instance().enableDebugLogging();
            Log::instance().enableStatusLogging();
        }else
        {
            // Log::instance().disableDebugLogging();
            Log::instance().disableStatusLogging();
        }     

        // Setup Vehicle
        vehicle_ = new Vehicle(_cfg.device.c_str(), _cfg.baudrate, true, _cfg.useAdvancedSensing);

        if (vehicle_ == nullptr)
        {
            std::cout << "\033[31mVehicle not initialized, exiting \033[m" << std::endl;
            return false;
        }

        // Check if the communication is working fine
        if (!vehicle_->protocolLayer->getDriver()->getDeviceStatus())
        {
            std::cout << "\033[31mComms appear to be incorrectly set up exiting \033[m" << std::endl;
            return false;
        }

        // Activate
        Vehicle::ActivateData activateData;
        activateData.ID = _cfg.app_id;
        char app_key[65];
        activateData.encKey = app_key;
        std::strcpy(activateData.encKey, _cfg.app_key.c_str());
        activateData.version = vehicle_->getFwVersion();

        ACK::ErrorCode ack = vehicle_->activate(&activateData, functionTimeout_);
        if (ACK::getError(ack))
        {
            ACK::getErrorCodeMessage(ack, __func__);
            std::cout << "\033[31mError validating Vehicle, exiting \033[m" << std::endl;
            return false;
        }

        std::cout << "\033[32mVehicle initialized \033[m" << std::endl;

        return true;
    }

    // ----------------------------------------------------------------------
    bool HAL::extract(Config& _cfg)
    {
        using namespace tinyxml2;
        
        XMLDocument doc;
	    auto error = doc.LoadFile("../config/config.xml");
        if (error != XML_SUCCESS)
            return false;

        auto configElement = doc.FirstChildElement("config_dal");
        
        auto uavElement = configElement->FirstChildElement("uav");
        auto appId = uavElement->FirstChildElement("app_id");
        auto appKey = uavElement->FirstChildElement("app_key");
        auto device = uavElement->FirstChildElement("device");
        auto baudrate = uavElement->FirstChildElement("baudrate");
        auto m600 = uavElement->FirstChildElement("m600");
        auto advSensing = uavElement->FirstChildElement("advanced_sensing");

        auto logsElement = configElement->FirstChildElement("logs");
        auto useLog = logsElement->FirstChildElement("use_log");
        auto djiLog = logsElement->FirstChildElement("dji_log");

        _cfg.app_id = static_cast<int>(appId->IntAttribute("value"));
        _cfg.app_key = static_cast<std::string>(appKey->Attribute("value"));
        _cfg.device = static_cast<std::string>(device->Attribute("value"));
        _cfg.baudrate = static_cast<int>(baudrate->IntAttribute("value"));
        _cfg.isM600 = static_cast<bool>(m600->BoolAttribute("value"));
        _cfg.useAdvancedSensing = static_cast<bool>(advSensing->BoolAttribute("value"));
        _cfg.useLog = static_cast<bool>(useLog->BoolAttribute("value"));
        _cfg.useLogDJI = static_cast<bool>(djiLog->BoolAttribute("value"));
        
        return true;
    }

    // ----------------------------------------------------------------------
    bool HAL::topicsMinimal()
    {
        if(obtainControlAuthority())
        {
            std::cout << "\033[32mUsing minimal topics \033[m" << std::endl;

            // TODO: THIS TOPICS ARE MINIMAL ??
            Topics topicsMinimal;
            topicsMinimal.push_back(TOPIC_GPS_DETAILS);
            topicsMinimal.push_back(TOPIC_HARD_SYNC);
            topicsMinimal.push_back(TOPIC_VELOCITY);
            topicsMinimal.push_back(TOPIC_GPS_FUSED);
            topicsMinimal.push_back(TOPIC_GPS_SIGNAL_LEVEL);
            topicsMinimal.push_back(TOPIC_ALTITUDE_FUSIONED);
            topicsMinimal.push_back(TOPIC_STATUS_FLIGHT);
            topicsMinimal.push_back(TOPIC_STATUS_DISPLAYMODE);
            topicsMinimal.push_back(TOPIC_BATTERY_INFO);
            topicsMinimal.push_back(TOPIC_RC);

            topics_ = topicsMinimal;
            if(extractTopics(topics_))
            {
                sleep(3); // Wait for the data to start coming in
                return setLocalPosition();
            }
            else
            {
                return false;
            } 
        }
        else
        {
            return false;
        }
    }

    // ----------------------------------------------------------------------
    bool HAL::topicsAll()
    {
        if(obtainControlAuthority())
        {
            std::cout << "\033[32mUsing all default topics \033[m" << std::endl;

            Topics topicsDefault;
            topicsDefault.push_back(TOPIC_GPS_DETAILS);
            topicsDefault.push_back(TOPIC_HARD_SYNC);
            topicsDefault.push_back(TOPIC_ANGULAR_RATE_FUSIONED);
            topicsDefault.push_back(TOPIC_QUATERNION);
            topicsDefault.push_back(TOPIC_VELOCITY);
            topicsDefault.push_back(TOPIC_GPS_FUSED);
            topicsDefault.push_back(TOPIC_GPS_SIGNAL_LEVEL);
            topicsDefault.push_back(TOPIC_ALTITUDE_FUSIONED);
            topicsDefault.push_back(TOPIC_COMPASS);
            topicsDefault.push_back(TOPIC_STATUS_FLIGHT);
            topicsDefault.push_back(TOPIC_STATUS_DISPLAYMODE);
            topicsDefault.push_back(TOPIC_BATTERY_INFO);
            topicsDefault.push_back(TOPIC_RC);
            if(!cfg_.isM600)
            {
                topicsDefault.push_back(TOPIC_RC_WITH_FLAG_DATA);    // NOT VALID FOR M600
                topicsDefault.push_back(TOPIC_RC_FULL_RAW_DATA);     // NOT VALID FOR M600
            }
            topicsDefault.push_back(TOPIC_ANGULAR_RATE_RAW);
            topicsDefault.push_back(TOPIC_ACCELERATION_RAW);

            // topics.push_back(TOPIC_CONTROL_DEVICE);

            topics_ = topicsDefault;
            if(extractTopics(topics_))
            {
                sleep(3); // Wait for the data to start coming in
                return setLocalPosition();
            }
            else
            {
                return false;
            } 
        }
        else
        {
            return false;
        }   
    }

    // ----------------------------------------------------------------------
    bool HAL::topicsCustom(Topics _topics)
    {
        if(obtainControlAuthority())
        {
            if(_topics.size() > 0)
            {
                std::cout << "\033[32mUsing Custom Topics \033[m" << std::endl;
                topics_ = _topics;
                if(extractTopics(topics_))
                {
                    sleep(3); // Wait for the data to start coming in
                    setLocalPosition();
                    return true;
                }
                else
                {
                    return false;
                }                
            }
            else
            {
                std::cout << "\033[32mCan not use Custom Topics \033[m" << std::endl;
                return false;
            }
        }
        else
        {
            return false;
        }   
    }

    // ----------------------------------------------------------------------
    bool HAL::obtainControlAuthority()
    {
        ACK::ErrorCode ctrlStatus = HAL::vehicle_->obtainCtrlAuthority(HAL::functionTimeout_);
        if (ACK::getError(ctrlStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(ctrlStatus, __func__);
            std::cout << "\033[31mNot obtained Control Authority, exiting \033[m" << std::endl;
            return false;
        }
        std::cout << "\033[32mObtained Control Authority of the Vehicle \033[m" << std::endl;
        return true;
    }

    // ----------------------------------------------------------------------
    bool HAL::setLocalPosition()
    {
        if (std::find(topics_.begin(), topics_.end(), TOPIC_ALTITUDE_FUSIONED) != topics_.end() && 
            std::find(topics_.begin(), topics_.end(), TOPIC_GPS_FUSED) != topics_.end())
        {
            originAltitude_ = vehicle_->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
            originGPS_ = vehicle_->subscribe->getValue<TOPIC_GPS_FUSED>();
            std::cout << "\033[32mSet local Position \033[m" << std::endl;
            return true;
        }
        else
        {
            std::cout << "\033[32mNot set local Position \033[m" << std::endl;
            return false;
        }
    }

    // ----------------------------------------------------------------------
    bool HAL::extractTopics(Topics _topics)
    {
        for (auto &e : _topics)
        {
            if (std::find(DefaultTopics400hz.begin(), DefaultTopics400hz.end(), e) != DefaultTopics400hz.end())
                topicsExtract400hz_.push_back(e);
            
            if (std::find(DefaultTopics200hz.begin(), DefaultTopics200hz.end(), e) != DefaultTopics200hz.end())
                topicsExtract200hz_.push_back(e);

            if (std::find(DefaultTopics50hz.begin(), DefaultTopics50hz.end(), e) != DefaultTopics50hz.end())
                topicsExtract50hz_.push_back(e);

            if (std::find(DefaultTopics5hz.begin(), DefaultTopics5hz.end(), e) != DefaultTopics5hz.end())
                topicsExtract5hz_.push_back(e);
        }

        bool result;
        result = subscribeToTopic(topicsExtract400hz_, 400);
        if(!result)
            return false;

        result = subscribeToTopic(topicsExtract200hz_, 200);
        if(!result)
            return false;

        result = subscribeToTopic(topicsExtract50hz_, 50);
        if(!result)
            return false;

        result = subscribeToTopic(topicsExtract5hz_, 5);
        if(!result)
            return false;

        return true;
    }
    
    // ----------------------------------------------------------------------
    bool HAL::subscribeToTopic(Topics _topics, int _freq)
    {
        ACK::ErrorCode subscribeStatus = vehicle_->subscribe->verify(functionTimeout_);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            std::cout << "\033[31mNot verified subscription to Telemetry, exiting \033[m" << std::endl;
            return false;
        }

        TopicName topicList[_topics.size()];   
        for (unsigned int i = 0; i < _topics.size(); i++ )
        {
            topicList[i] = _topics[i];
        }

        int numTopic = sizeof(topicList) / sizeof(topicList[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList, enableTimestamp, _freq);
        if (!pkgStatus)
        {
            std::cout << "\033[31mNot init package \033[m" << pkgIndex_ << "\033[31m from topic list, exiting \033[m" << std::endl;
            return false;
        }

        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            unsubscribeAllTopics(); 
            std::cout << "\033[31mStart package \033[m" << pkgIndex_ << "\033[31m error, exiting \033[m" << std::endl;
            return false;
        }

        pkgIndex_++;
        
        return true;
    }
    
    // ----------------------------------------------------------------------
    void HAL::unsubscribeAllTopics()
    {
        for(int i = 0; i < pkgIndex_; i++ )
        {
            ACK::ErrorCode ack = vehicle_->subscribe->removePackage(i, functionTimeout_);
            if (ACK::getError(ack))
            {
                std::cout << "\033[31mError unsubscribing at package: \033[m" + std::to_string(i) + "\033[31mplease restart the drone/FC to get back to a clean state, exiting \033[m" << std::endl;
            }
        }
    }

    // ----------------------------------------------------------------------
    void HAL::showLogo()
    {
        std::cout <<    "\n"
                        "\033[34m     ........               ........    \n"
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
                        " |____________________________________| \033[m\n\n";
    }

    // ----------------------------------------------------------------------
    void HAL::showCfg(Config _cfg)
    {
        std::cout << "Information extracted from Config.xml:" << std::endl;
        std::cout << "- ID: " << _cfg.app_id << std::endl;
        std::cout << "- Key: " << _cfg.app_key << std::endl;
        std::cout << "- Device: " << _cfg.device << std::endl;
        std::cout << "- Baudrate: " << _cfg.baudrate << std::endl;
        std::cout << "- M600 type: " << _cfg.isM600 << std::endl;
        std::cout << "- Log|DJI Log: " << _cfg.useLog << " | " << _cfg.useLogDJI << std::endl;
        std::cout << std::endl;
    }

}
