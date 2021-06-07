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

#include <dal/dji/MissionsDJI.h>

namespace dal{
    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    MissionsDJI::MissionsDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    MissionsDJI::~MissionsDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR MISSIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool MissionsDJI::positionGPS(Eigen::Vector3f _wayPoint, dataMission _config){

        DJI::OSDK::WayPointInitSettings fdata;
        setWaypointInitDefaults(&fdata);

        fdata.maxVelocity = _config.maxVelWP;
        fdata.idleVelocity = _config.idleVelWP;
        fdata.finishAction = _config.finishActionWP;
        fdata.executiveTimes = _config.executiveTimesWP;
        fdata.yawMode = _config.yawMode;
        fdata.traceMode = _config.traceModeWP;
        fdata.RCLostAction = _config.rcLostWP;
        fdata.indexNumber = 2;

        DJI::OSDK::ACK::ErrorCode initAck = HAL::vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT, HAL::functionTimeout_, &fdata);
        if (DJI::OSDK::ACK::getError(initAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
            std::cout << "\033[31mError at init waypoint GPS, exiting \033[m" << std::endl;
            return false;
        }
        
        // Get actual Lat and Lon
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type gps = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();

        float altitude = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
        float altitudeWP = altitude - HAL::originAltitude_;
        
        // Create Waypoints
        DJI::OSDK::WayPointSettings wp1;
        setWaypointDefaults(&wp1);

        wp1.turnMode = _config.turnModeWP;
        wp1.index     = 0;
        wp1.latitude  = gps.latitude;
        wp1.longitude = gps.longitude;
        wp1.altitude  = altitudeWP;
        
        DJI::OSDK::WayPointSettings wp2;
        setWaypointDefaults(&wp2);

        wp2.turnMode = _config.turnModeWP;
        wp2.index     = 1;
        wp2.latitude  = _wayPoint[0];
        wp2.longitude = _wayPoint[1];
        wp2.altitude  = _wayPoint[2];

        std::vector<DJI::OSDK::WayPointSettings> generatedWaypts;
        generatedWaypts.push_back(wp1);
        generatedWaypts.push_back(wp2);

        // Upload Waypoints
        uploadWaypoints(generatedWaypts);

        HAL::vehicle_->missionManager->printInfo();

        DJI::OSDK::ACK::ErrorCode startAck = HAL::vehicle_->missionManager->wpMission->start(HAL::functionTimeout_);
        if (DJI::OSDK::ACK::getError(startAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
            std::cout << "\033[31mError at start waypoint GPS, exiting \033[m" << std::endl;
            return false;
        }else{
            std::cout << "\033[32mGoing to waypoint GPS \033[m" << std::endl;
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool MissionsDJI::mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config){

        if(_config.missionType == "waypoint"){

            // WAYPOINTS MISSION
            missionType_ = "waypoint";

            // Waypoint Mission : Initialization
            DJI::OSDK::WayPointInitSettings fdata;
            setWaypointInitDefaults(&fdata);

            fdata.maxVelocity = _config.maxVelWP;
            fdata.idleVelocity = _config.idleVelWP;
            fdata.finishAction = _config.finishActionWP;
            fdata.executiveTimes = _config.executiveTimesWP;
            fdata.yawMode = _config.yawMode;
            fdata.traceMode = _config.traceModeWP;
            fdata.RCLostAction = _config.rcLostWP;
            fdata.indexNumber = _wayPoints.size() + 2; // We add 1 to get the aircarft back to the start and pose from init

            DJI::OSDK::ACK::ErrorCode initAck = HAL::vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT, HAL::functionTimeout_, &fdata);
            if (DJI::OSDK::ACK::getError(initAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
                std::cout << "\033[31mError at init mission manager, exiting \033[m" << std::endl;
                return false;
            }

            std::cout << "\033[32mInitializing Waypoint Mission... \033[m" << std::endl;

            // Waypoint Mission: Create Waypoints
            std::vector<WayPointSettings> generatedWaypts = createWaypoints(_wayPoints, _config);
            std::cout << "\033[32mCreating Waypoints... \033[m" << std::endl;

            // Waypoint Mission: Upload the waypoints
            uploadWaypoints(generatedWaypts);
            std::cout << "\033[32mUploading Waypoints... \033[m" << std::endl;

            HAL::vehicle_->missionManager->printInfo();

        }else if(_config.missionType == "hotpoint"){

            // HOTPOINT MISSION
            missionType_ = "hotpoint";

            // Hotpoint Mission Initialize
            HAL::vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT, HAL::functionTimeout_, NULL);

            HAL::vehicle_->missionManager->hpMission->setHotPoint(_wayPoints[0][1], _wayPoints[0][0], _wayPoints[0][2]);

            HAL::vehicle_->missionManager->hpMission->setRadius(_config.radiusHP);

            HAL::vehicle_->missionManager->printInfo();

        }else{
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool MissionsDJI::start_mission(){

        if(missionType_ == "waypoint"){

            DJI::OSDK::ACK::ErrorCode startAck = HAL::vehicle_->missionManager->wpMission->start(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(startAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
                std::cout << "\033[31mError at start mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mStarting Waypoint Mission... \033[m" << std::endl;
            }

        }else if(missionType_ == "hotpoint"){

            DJI::OSDK::ACK::ErrorCode startAck = HAL::vehicle_->missionManager->hpMission->start(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(startAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
                std::cout << "\033[31mError at start mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mStarting Hotpoint Mission... \033[m" << std::endl;
            }

        }else{
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool MissionsDJI::pause_mission(){

        if(missionType_ == "waypoint"){
            
            DJI::OSDK::ACK::ErrorCode pauseAck = HAL::vehicle_->missionManager->wpMission->pause(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(pauseAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(pauseAck, __func__);
                std::cout << "\033[31mError at pause mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mPause Waypoint Mission... \033[m" << std::endl;
            }

        }else if(missionType_ == "hotpoint"){

            DJI::OSDK::ACK::ErrorCode pauseAck = HAL::vehicle_->missionManager->hpMission->pause(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(pauseAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(pauseAck, __func__);
                std::cout << "\033[31mError at pause mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mPause Hotpoint Mission... \033[m" << std::endl;
            }

        }else{
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool MissionsDJI::stop_mission(){

        if(missionType_ == "waypoint"){

            DJI::OSDK::ACK::ErrorCode stopAck = HAL::vehicle_->missionManager->wpMission->stop(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(stopAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(stopAck, __func__);
                std::cout << "\033[31mError at stop mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mStop Waypoint Mission... \033[m" << std::endl;
            }
        }else if(missionType_ == "hotpoint"){
            
            DJI::OSDK::ACK::ErrorCode stopAck = HAL::vehicle_->missionManager->hpMission->stop(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(stopAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(stopAck, __func__);
                std::cout << "\033[31mError at stop mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mStop Hotpoint Mission... \033[m" << std::endl;
            }

        }else{
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool MissionsDJI::resume_mission(){

        if(missionType_ == "waypoint"){

            DJI::OSDK::ACK::ErrorCode resumeAck = HAL::vehicle_->missionManager->wpMission->resume(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(resumeAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(resumeAck, __func__);
                std::cout << "\033[31mError at resume mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mResume Waypoint Mission... \033[m" << std::endl;
            }
        }else if(missionType_ == "hotpoint"){

            DJI::OSDK::ACK::ErrorCode resumeAck = HAL::vehicle_->missionManager->hpMission->resume(HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(resumeAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(resumeAck, __func__);
                std::cout << "\033[31mError at resume mission, exiting \033[m" << std::endl;
                return false;
            }else{
                std::cout << "\033[32mResume Hotpoint Mission... \033[m" << std::endl;

            }
        }else{
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    void MissionsDJI::setWaypointDefaults(DJI::OSDK::WayPointSettings* _wp){

        _wp->damping         = 0;
        _wp->yaw             = 0;
        _wp->gimbalPitch     = 0;
        _wp->turnMode        = 0;
        _wp->hasAction       = 0;
        _wp->actionTimeLimit = 100;
        _wp->actionNumber    = 0;
        _wp->actionRepeat    = 0;
        for (int i = 0; i < 16; ++i){
            _wp->commandList[i]      = 0;
            _wp->commandParameter[i] = 0;
        }
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    void MissionsDJI::setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* _fdata){

        _fdata->maxVelocity    = 2;
        _fdata->idleVelocity   = 1;
        _fdata->finishAction   = 0;
        _fdata->executiveTimes = 0;
        _fdata->yawMode        = 0;
        _fdata->traceMode      = 0;
        _fdata->RCLostAction   = 0;
        _fdata->gimbalPitch    = 0;
        _fdata->latitude       = 0;
        _fdata->longitude      = 0;
        _fdata->altitude       = 0;
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<DJI::OSDK::WayPointSettings> MissionsDJI::createWaypoints(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config){
        
        // Get actual Lat and Lon
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type gps = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();

        // Let's create a vector to store our waypoints in
        std::vector<DJI::OSDK::WayPointSettings> wp_list;

        // Init wp
        DJI::OSDK::WayPointSettings init_wp;
        setWaypointDefaults(&init_wp);

        init_wp.turnMode = _config.turnModeWP;
        init_wp.index     = 0;
        init_wp.latitude  = gps.latitude;
        init_wp.longitude = gps.longitude;
        init_wp.altitude  = _wayPoints[0][2];

        wp_list.push_back(init_wp);   

        for (unsigned i = 0; i < _wayPoints.size(); i++){

            DJI::OSDK::WayPointSettings  wp;
            setWaypointDefaults(&wp);

            wp.turnMode = _config.turnModeWP;
            wp.index     = i+1;
            wp.latitude  = _wayPoints[i][0];
            wp.longitude = _wayPoints[i][1];
            wp.altitude  = _wayPoints[i][2];
            wp_list.push_back(wp);

        }

        return wp_list;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void MissionsDJI::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& _wpList){

        for (std::vector<DJI::OSDK::WayPointSettings>::iterator wp = _wpList.begin(); wp != _wpList.end(); ++wp){
            std::cout << "\033[32mWaypoint created at (LLA): \033[m" << std::to_string(wp->latitude) + "\033[32m | \033[m" + std::to_string(wp->longitude) + "\033[32m | \033[m" + std::to_string(wp->altitude) << std::endl;

            DJI::OSDK::ACK::WayPointIndex wpDataACK = HAL::vehicle_->missionManager->wpMission->uploadIndexData(&(*wp), HAL::functionTimeout_);
            if (DJI::OSDK::ACK::getError(wpDataACK.ack)){
                DJI::OSDK::ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
            }
        }
    }

}
