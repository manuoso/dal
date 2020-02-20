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


#include <dal/backends/BackendDJI.h>

namespace dal{
    //-----------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------------------
    BackendDJI::BackendDJI(){}

    //-----------------------------------------------------------------------------------------------------------------
    BackendDJI::~BackendDJI(){

        unsubscribeAllTopics();

        if(useLogStatus_){
            LogStatus::get()->close();
        }

        delete vehicle_;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR CONTROL
    //---------------------------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::emergencyBrake(){

        secureGuard_.lock();
        vehicle_->control->emergencyBrake();
        secureGuard_.unlock();

        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::recoverFromManual(){
        
        // Obtain Control Authority
        return obtainControlAuthority(true);
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::takeOff(const float _height){

        // Start takeoff
        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode takeoffStatus = vehicle_->control->takeoff(functionTimeout_);
        secureGuard_.unlock();
        if(DJI::OSDK::ACK::getError(takeoffStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(takeoffStatus, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Error at start takeoff, exiting", true);
            }
            return false;
        }

        // First check: Motors started
        int motorsNotStarted = 0;
        int timeoutCycles    = 20;

        while(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() != DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND &&
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START &&
            motorsNotStarted < timeoutCycles){
                
                motorsNotStarted++;
                usleep(100000);
        }

        if(motorsNotStarted == timeoutCycles){
            if(useLogStatus_){
                LogStatus::get()->error("Takeoff failed. Motors are not spinning, exiting", true);
            }
            return false; 
        }else{
            if(useLogStatus_){
                LogStatus::get()->status("Motors spinning...", true);     
            }
        }
      
        // Second check: In air
        int stillOnGround = 0;
        timeoutCycles     = 110;

        while(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR &&
            (vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
            stillOnGround < timeoutCycles){
            
            stillOnGround++;
            usleep(100000);
        }

        if(stillOnGround == timeoutCycles){
            if(useLogStatus_){
                LogStatus::get()->error("Takeoff failed. Aircraft is still on the ground, but the motors are spinning, exiting", true);
            }
            return false;
        }else{
            if(useLogStatus_){
                LogStatus::get()->status("Ascending...", true);
            }        
        }

        // Final check: Finished takeoff
        while(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF){
                
                sleep(1);
        }

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
                
                if(useLogStatus_){
                    LogStatus::get()->status("Successful takeoff!", true);
                }
        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting", true);
            }
            return false;
        }

        
        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::land(){

        // Start landing
        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode landingStatus = vehicle_->control->land(functionTimeout_);
        secureGuard_.unlock();
        if(DJI::OSDK::ACK::getError(landingStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(landingStatus, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Error at land, exiting", true);
            }
            return false;
        }

        // First check: Landing started
        int landingNotStarted = 0;
        int timeoutCycles     = 20;

        while (vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            landingNotStarted < timeoutCycles){
                
                landingNotStarted++;
                usleep(100000);
        }

        if(landingNotStarted == timeoutCycles){
            if(useLogStatus_){
                LogStatus::get()->error("Landing failed. Aircraft is still in the air, exiting", true);
            }
            return false;
        }else{
            if(useLogStatus_){
                LogStatus::get()->status("Landing...", true);
            }
        }

        // Second check: Finished landing
        while (vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR){
                
                sleep(1);
        }

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
                
                if(useLogStatus_){
                    LogStatus::get()->status("Successful landing!", true);
                }
        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting", true);
            }
            return false;
        }
        
        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::positionCtrlYaw(float _x, float _y, float _z, float _yaw){

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            if(useLogStatus_){
                LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            }
            return false;
        }

        if(useLogStatus_){
            LogStatus::get()->status("Moving in local position", false);
        }
        
        // Get local position
        VectorLocalPosition localPose;
        getLocalPosition(localPose);
        
        // Get initial offset. We will update this in a loop later.
        double xOffset = _x - localPose(0);
        double yOffset = _y - localPose(1);
        double zOffset = _z - localPose(2);

        // 0.1 m or 10 cms is the minimum error to reach target in x, y and z axes.
        // This error threshold will have to change depending on aircraft / payload / wind conditions
        double xCmd, yCmd, zCmd;
        if(((std::abs(xOffset)) < 0.1) && ((std::abs(yOffset)) < 0.1) && (localPose(2) > (zOffset - 0.1)) && (localPose(2) < (zOffset + 0.1))){
            xCmd = 0;
            yCmd = 0;
            zCmd = 0;
        }else{
            xCmd = xOffset;
            yCmd = yOffset;
            zCmd = _z;
        }

        // 666 TODO: YAW NOT IMPLEMENTED!

        secureGuard_.lock();
        vehicle_->control->positionAndYawCtrl(xCmd, yCmd, zCmd, _yaw);
        secureGuard_.unlock();

        return true;
    }
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate){

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            if(useLogStatus_){
                LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            }
            return false;
        }
        
        if(useLogStatus_){
            LogStatus::get()->status("Moving in velocity", false);
        }

        secureGuard_.lock();
        vehicle_->control->velocityAndYawRateCtrl(_vx, _vy, _vz, _yawRate);
        secureGuard_.unlock();
        
        return true;
    }    


    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::attitudeCtrlVer(float _roll, float _pitch, float _yaw, float _z){

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            if(useLogStatus_){
                LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            }
            return false;
        }
        
        if(useLogStatus_){
            LogStatus::get()->status("Moving in attitude and vertical position", false);
        }

        secureGuard_.lock();
        vehicle_->control->attitudeAndVertPosCtrl(_roll, _pitch, _yaw, _z);
        secureGuard_.unlock();
        
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::angularRateCtrlVer(float _rollRate, float _pitchRate, float _yawRate, float _z){

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            if(useLogStatus_){
                LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            }
            return false;
        }
        
        if(useLogStatus_){
            LogStatus::get()->status("Moving in attitude rate and vertical position", false);
        }

        secureGuard_.lock();
        vehicle_->control->angularRateAndVertPosCtrl(_rollRate, _pitchRate, _yawRate, _z);
        secureGuard_.unlock();
        
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::selfControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP){

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            if(useLogStatus_){
                LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            }
            return false;
        }

        if(useLogStatus_){
            LogStatus::get()->status("Moving in self control", false);
        }

        DJI::OSDK::Control::CtrlData ctrlData(_flag, RAD2DEG(_xSP), RAD2DEG(_ySP), _zSP, RAD2DEG(_yawSP));
        secureGuard_.lock();
        vehicle_->control->flightCtrl(ctrlData);
        secureGuard_.unlock();

        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::positionCtrlGPS(Eigen::Vector3f _wayPoint, dataMission _config){

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

        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode initAck = vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT, functionTimeout_, &fdata);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(initAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Error at init waypoint GPS, exiting", true);
            }
            return false;
        }

        // Create Waypoints
        std::vector<DJI::OSDK::WayPointSettings> generatedWaypts;

        DJI::OSDK::WayPointSettings  wp1;
        setWaypointDefaults(&wp1);

        // Get actual Lat and Lon
        Eigen::Vector2f gps;
        getGPS(gps);

        float altitude = 0;
        getAltitude(altitude);

        wp1.turnMode = _config.turnModeWP;
        wp1.index     = 0;
        wp1.latitude  = gps(0);
        wp1.longitude = gps(1);
        wp1.altitude  = altitude;
        generatedWaypts.push_back(wp1);
        
        DJI::OSDK::WayPointSettings  wp2;
        setWaypointDefaults(&wp2);

        wp2.turnMode = _config.turnModeWP;
        wp2.index     = 1;
        wp2.latitude  = _wayPoint(0);
        wp2.longitude = _wayPoint(1);
        wp2.altitude  = _wayPoint(2);
        generatedWaypts.push_back(wp2);

        // Upload Waypoints
        uploadWaypoints(generatedWaypts);

        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode startAck = vehicle_->missionManager->wpMission->start(functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(startAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Error at start waypoint GPS, exiting", true);
            }
            return false;
        }else{
            if(useLogStatus_){
                LogStatus::get()->status("Going to waypoint GPS", true);
            }
        }

        return true;

    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR MISSIONS
    //---------------------------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::mission(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config){

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
            fdata.indexNumber = _wayPoints.size() + 1; // We add 1 to get the aircarft back to the start

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode initAck = vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT, functionTimeout_, &fdata);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(initAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at init mission manager, exiting", true);
                }
                return false;
            }

            secureGuard_.lock();
            vehicle_->missionManager->printInfo();
            secureGuard_.unlock();
            if(useLogStatus_){
                LogStatus::get()->status("Initializing Waypoint Mission...", true);
            }

            // Waypoint Mission: Create Waypoints
            std::vector<WayPointSettings> generatedWaypts = createWaypoints(_wayPoints, _config);
            if(useLogStatus_){
                LogStatus::get()->status("Creating Waypoints...", true);
            }

            // Waypoint Mission: Upload the waypoints
            uploadWaypoints(generatedWaypts);
            if(useLogStatus_){
                LogStatus::get()->status("Uploading Waypoints...", true);
            }

        }else if(_config.missionType == "hotpoint"){

            // HOTPOINT MISSION
            missionType_ = "hotpoint";

            // Hotpoint Mission Initialize
            secureGuard_.lock();
            vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT, functionTimeout_, NULL);
            secureGuard_.unlock();

            secureGuard_.lock();
            vehicle_->missionManager->printInfo();
            secureGuard_.unlock();

            secureGuard_.lock();
            vehicle_->missionManager->hpMission->setHotPoint(_wayPoints[0](1), _wayPoints[0](0), _wayPoints[0](2));
            secureGuard_.unlock();

            secureGuard_.lock();
            vehicle_->missionManager->hpMission->setRadius(_config.radiusHP);
            secureGuard_.unlock();

        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Unrecognised mission type, exiting", true);
            }
            return false;
        }
        

        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::start_mission(){

        if(missionType_ == "waypoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode startAck = vehicle_->missionManager->wpMission->start(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(startAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at start mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Starting Waypoint Mission...", true);
                }
            }

        }else if(missionType_ == "hotpoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode startAck = vehicle_->missionManager->hpMission->start(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(startAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at start mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Starting Hotpoint Mission...", true);
                }
            }

        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Unrecognised mission type, exiting", true);
            }
            return false;
        }

        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::pause_mission(){

        if(missionType_ == "waypoint"){
            
            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode pauseAck = vehicle_->missionManager->wpMission->pause(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(pauseAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(pauseAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at pause mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Pause Waypoint Mission...", true);
                }
            }

        }else if(missionType_ == "hotpoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode pauseAck = vehicle_->missionManager->hpMission->pause(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(pauseAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(pauseAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at pause mission, exiting", true); 
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Pause Hotpoint Mission...", true);
                }
            }

        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Unrecognised mission type, exiting", true);
            }
            return false;
        }

        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::stop_mission(){

        if(missionType_ == "waypoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode stopAck = vehicle_->missionManager->wpMission->stop(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(stopAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(stopAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at stop mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Stop Waypoint Mission...", true);
                }
            }

        }else if(missionType_ == "hotpoint"){
            
            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode stopAck = vehicle_->missionManager->hpMission->stop(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(stopAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(stopAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at stop mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Stop Hotpoint Mission...", true);
                }
            }

        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Unrecognised mission type, exiting", true);
            }
            return false;
        }

        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::resume_mission(){

        if(missionType_ == "waypoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode resumeAck = vehicle_->missionManager->wpMission->resume(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(resumeAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(resumeAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at resume mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Resume Waypoint Mission...", true);
                }
            }

        }else if(missionType_ == "hotpoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode resumeAck = vehicle_->missionManager->hpMission->resume(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(resumeAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(resumeAck, __func__);
                if(useLogStatus_){
                    LogStatus::get()->error("Error at resume mission, exiting", true);
                }
                return false;
            }else{
                if(useLogStatus_){
                    LogStatus::get()->status("Resume Hotpoint Mission...", true);
                }
            }

        }else{
            if(useLogStatus_){
                LogStatus::get()->error("Unrecognised mission type, exiting", true);
            }
            return false;
        }

        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR TELEMETRY
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    // INITS

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendDJI::initLocalPosition(){
        return setLocalPosition();
    }

    //---------------------------------------------------------------------------------------------------------------------
    // GETTERS

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getPositionVO(VectorPositionVO& _data){
        
        secureGuard_.lock();
        position_vo_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_POSITION_VO>();
        secureGuard_.unlock();

        _data(0) = position_vo_.x;
        _data(1) = position_vo_.y;
        _data(2) = position_vo_.z;
        _data(3) = position_vo_.xHealth;
        _data(4) = position_vo_.yHealth;
        _data(5) = position_vo_.zHealth;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getGPS(VectorGPS& _data){
        
        secureGuard_.lock();
        latLon_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        secureGuard_.unlock();

        _data(0) = latLon_.latitude;
        _data(1) = latLon_.longitude; 

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getGPSDetail(VectorGPSDetail& _data){

        secureGuard_.lock();
        GPSDetail_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS>();
        secureGuard_.unlock();

        _data(0) = GPSDetail_.fix;
        _data(1) = GPSDetail_.gnssStatus;
        _data(2) = GPSDetail_.hacc;
        _data(3) = GPSDetail_.sacc;
        _data(4) = GPSDetail_.usedGPS;
        _data(5) = GPSDetail_.usedGLN; 
        _data(6) = GPSDetail_.NSV;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getGPSSignal(int& _data){

        secureGuard_.lock();
        GPSSignal_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL>();
        secureGuard_.unlock();

        _data = GPSSignal_;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getAltitude(float& _data){
        
        secureGuard_.lock();
        altitude_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
        secureGuard_.unlock();

        actualAltitude_ = altitude_ - originAltitude_;

        _data = actualAltitude_;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getAngularRate(VectorAngularRate& _data){
        
        secureGuard_.lock();
        angularRate_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();
        secureGuard_.unlock();

        _data(0) = angularRate_.x;
        _data(1) = angularRate_.y;
        _data(2) = angularRate_.z;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getHardSync(VectorHardSync& _data){
        
        secureGuard_.lock();
        hardSync_FC_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HARD_SYNC>();
        secureGuard_.unlock();    

        _data(0) = hardSync_FC_.w.x;
        _data(1) = hardSync_FC_.w.y;
        _data(2) = hardSync_FC_.w.z;
        _data(3) = hardSync_FC_.a.x;
        _data(4) = hardSync_FC_.a.y;
        _data(5) = hardSync_FC_.a.z;
        _data(6) = hardSync_FC_.q.q0;
        _data(7) = hardSync_FC_.q.q1;
        _data(8) = hardSync_FC_.q.q2;
        _data(9) = hardSync_FC_.q.q3;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getCompass(VectorCompass& _data){

        secureGuard_.lock();
        compass_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_COMPASS>();
        secureGuard_.unlock();

        _data(0) = compass_.x;
        _data(1) = compass_.y;
        _data(2) = compass_.z;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getQuaternion(VectorQuaternion& _data){

        secureGuard_.lock();
        quaternion_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();
        secureGuard_.unlock();

        _data(0) = quaternion_.q0; 
        _data(1) = quaternion_.q1; 
        _data(2) = quaternion_.q2; 
        _data(3) = quaternion_.q3; 

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getVelocity(VectorVelocity& _data){
        
        secureGuard_.lock();
        velocity_  = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
        secureGuard_.unlock();

        _data(0) = velocity_.data.x; 
        _data(1) = velocity_.data.y; 
        _data(2) = velocity_.data.z;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getStatusFlight(std::string& _data){
        
        secureGuard_.lock();
        flightStatus_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        secureGuard_.unlock();

        std::string sFlightStatus, sMode;
        if(flightStatus_ == 0){
            sFlightStatus = "STOPED";
        }else if(flightStatus_ == 1){
            sFlightStatus = "ON_GROUND";
        }else if(flightStatus_ == 2){
            sFlightStatus = "IN_AIR";
        }else{
            sFlightStatus = "UNRECOGNIZED";
        }

        _data = sFlightStatus; 

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getDisplayMode(std::string& _data){
        
        secureGuard_.lock();
        mode_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        secureGuard_.unlock();

        std::string sMode;
        if(mode_ == 0){
            sMode = "MANUAL";
        }else if(mode_ == 1){
            sMode = "ATTITUDE";
        }else if(mode_ == 6){
            sMode = "P_GPS";
        }else if(mode_ == 9){
            sMode = "HOT_POINT";
        }else if(mode_ == 10){
            sMode = "ASSISTED_TAKEOFF";
        }else if(mode_ == 11){
            sMode = "AUTO_TAKEOFF";
        }else if(mode_ == 12){
            sMode = "ASSISTED_LAND";
        }else if(mode_ == 15){
            sMode = "GO_HOME";
        }else if(mode_ == 17){
            sMode = "NAVI_SDK_CTMODE_CTRLRL";
        }else if(mode_ == 33){
            sMode = "FORCE_AUTO_LANDING";
        }else if(mode_ == 40){
            sMode = "SEARCH";
        }else if(mode_ == 41){
            sMode = "ENGINE_START";
        }else{
            sMode = "UNRECOGNIZED";
        }

        _data = sMode;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getBatery(int& _data){

        secureGuard_.lock();
        battery_info_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();
        secureGuard_.unlock();  

        _data = battery_info_.voltage;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getRC(VectorRC& _data){
        secureGuard_.lock();
        rc_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA>();
        secureGuard_.unlock();

        _data(0) = rc_.pitch;
        _data(1) = rc_.roll;
        _data(2) = rc_.yaw;
        _data(3) = rc_.throttle;
        _data(4) = rc_.flag.logicConnected;
        _data(5) = rc_.flag.skyConnected;
        _data(6) = rc_.flag.groundConnected;
        _data(7) = rc_.flag.appConnected;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getControlDevice(VectorControlDevice& _data){

        secureGuard_.lock();
        controlDevice_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();
        secureGuard_.unlock();

        _data(0) = controlDevice_.controlMode;
        _data(1) = controlDevice_.deviceStatus;
        _data(2) = controlDevice_.flightStatus;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::getLocalPosition(VectorLocalPosition& _data){

        secureGuard_.lock();
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        secureGuard_.unlock();

        localPoseFromGps(localPose_, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&originGPS_));

        _data(0) = localPose_.x;
        _data(1) = localPose_.y;
        _data(2) = localPose_.z;

        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::init(const Config &_config){

        std::cout <<    "     ........               ........    \n"
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
                        " |____________________________________| \n";

        // Use Log Status
        useLogStatus_ = _config.useLogStatus;

        if(useLogStatus_){
            LogStatus::init("DJIStatus" + std::to_string(time(NULL)));
        }

        // Setup Vehicle
        secureGuard_.lock();
        vehicle_ = new DJI::OSDK::Vehicle(_config.device.c_str(), _config.baudrate, true, _config.useAdvancedSensing);

        if(vehicle_ == NULL){
            if(useLogStatus_){
                LogStatus::get()->error("Vehicle not initialized, exiting", true);
            }
            return false;
        }

        // Check if the communication is working fine
        if(!vehicle_->protocolLayer->getDriver()->getDeviceStatus()){
            if(useLogStatus_){
                LogStatus::get()->error("Comms appear to be incorrectly set up exiting", true);
            }
            return false;
        }
        secureGuard_.unlock();

        // Activate
        DJI::OSDK::Vehicle::ActivateData activateData;
        activateData.ID = _config.app_id;
        char app_key[65];
        activateData.encKey = app_key;
        std::strcpy(activateData.encKey, _config.app_key.c_str());

        secureGuard_.lock();
        activateData.version = vehicle_->getFwVersion();
        DJI::OSDK::ACK::ErrorCode ack = vehicle_->activate(&activateData, functionTimeout_);
        secureGuard_.unlock();

        if(DJI::OSDK::ACK::getError(ack)){
            DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Error validating Vehicle, exiting", true);
            }
            return false;
        }

        if(useLogStatus_){
            LogStatus::get()->status("Vehicle initialized", true);
        }

        // Obtain Control Authority
        if(obtainControlAuthority(true)){
            if(_config.topics.size() > 0){
                if(useLogStatus_){
                    LogStatus::get()->status("Using std map", true);
                }
                
                bool result;
                int searchFreq = 0, searchFreqLast = 0, contMaxFreq = 0;
                std::map<DJI::OSDK::Telemetry::TopicName, int>::iterator it, itSameFreq;
                std::map<DJI::OSDK::Telemetry::TopicName, int> copyConfigTopics = _config.topics;
                it = copyConfigTopics.begin();

                while(it != copyConfigTopics.end()){ 
                    searchFreq = it->second;

                    if(searchFreqLast == searchFreq){
                        continue;
                    }

                    contMaxFreq++;
                    if(contMaxFreq > 5){
                        LogStatus::get()->status("Using more packages with different frequencies than DJI allows", true);
                        return false;
                    }

                    itSameFreq = it;
                    std::vector<DJI::OSDK::Telemetry::TopicName> topics;
                    while(itSameFreq != copyConfigTopics.end()){ 
                        if(searchFreq == itSameFreq->second){
                            topics.push_back(itSameFreq->first);
                            it = copyConfigTopics.erase(itSameFreq);
                        }

                        itSameFreq++;
                    }
                
                    if(topics.size() > 0){
                        result = subscribeToTopic(topics, searchFreq);
                        if(!result){
                            return false;
                        }
                    }

                    searchFreqLast = searchFreq;
                    if(copyConfigTopics.size() == 0){
                        break;
                    }else{
                        it++;
                    }

                }
                
                // Wait for the data to start coming in.
                sleep(3);

                return setLocalPosition();

            }else{
                // 666 TODO: NO SOPORTADO DEBIDO A QUE SOLO SE PUEDEN ENVIAR 5 PAQUETES MAXIMO DE TELEMETRIA
                if(useLogStatus_){
                    LogStatus::get()->status("Not working without std map", true);
                }
                return false;
            }

            return true;
        }else{
            return false;
        }    
    
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::obtainControlAuthority(bool _info){ 

        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode ctrlStatus = vehicle_->obtainCtrlAuthority(functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(ctrlStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(ctrlStatus, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Not obtained Control Authority, exiting", _info);
            }
            return false;
        }
        if(useLogStatus_){
            LogStatus::get()->status("Obtained Control Authority of the Vehicle", _info);
        }
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::subscribeToTopic(std::vector<DJI::OSDK::Telemetry::TopicName> _topics, int _freq){

        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode subscribeStatus = vehicle_->subscribe->verify(functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            if(useLogStatus_){
                LogStatus::get()->error("Not verified subscription to Telemetry, exiting", true);
            }
            return false;
        }

        DJI::OSDK::Telemetry::TopicName topicList[_topics.size()];   
        for(unsigned int i = 0; i < _topics.size(); i++ ){
            topicList[i] = _topics[i];
        }

        int numTopic = sizeof(topicList) / sizeof(topicList[0]);
        bool enableTimestamp = false;

        secureGuard_.lock();
        bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList, enableTimestamp, _freq);
        secureGuard_.unlock();
        if (!pkgStatus){
            if(useLogStatus_){
                LogStatus::get()->error("Not init package 0 from topic list, exiting", true);
            }
            return false;
        }

        secureGuard_.lock();
        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeAllTopics(); 
            if(useLogStatus_){
                LogStatus::get()->error("Start package 0 error, exiting", true);
            }
            return false;
        }

        pkgIndex_++;
        
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::unsubscribeAllTopics(){

        for(int i = 0; i < pkgIndex_; i++ ){
            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode ack = vehicle_->subscribe->removePackage(i, functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(ack)){
                if(useLogStatus_){
                    LogStatus::get()->error("Error unsubscribing at package: " + std::to_string(i) + "; please restart the drone/FC to get back to a clean state, exiting", true);
                }
                return false;
            }
        }
        
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::setLocalPosition(){

        secureGuard_.lock();
        originGPS_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        secureGuard_.unlock();
    
        secureGuard_.lock();
        originAltitude_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
        secureGuard_.unlock();
        
        return true;

    }
    
    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::localPoseFromGps(DJI::OSDK::Telemetry::Vector3f& _delta, void* _target, void* _origin){
    
        DJI::OSDK::Telemetry::GPSFused* subscriptionTarget = (DJI::OSDK::Telemetry::GPSFused*)_target;
        DJI::OSDK::Telemetry::GPSFused*  subscriptionOrigin = (DJI::OSDK::Telemetry::GPSFused*)_origin;
        
        double t_lon = subscriptionTarget->longitude * 180.0 / C_PI;
        double r_lon = subscriptionOrigin->longitude * 180.0 / C_PI;
        
        double t_lat = subscriptionTarget->latitude * 180.0 / C_PI;
        double r_lat = subscriptionOrigin->latitude * 180.0 / C_PI;

        double deltaLon   = t_lon - r_lon;
        double deltaLat   = t_lat - r_lat;

        // NEU -> North East Up
        _delta.y = DEG2RAD(deltaLon) * C_EARTH * cos(DEG2RAD(t_lat));
        _delta.x = DEG2RAD(deltaLat) * C_EARTH;
        _delta.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;

    }

    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::setWaypointDefaults(DJI::OSDK::WayPointSettings* _wp){

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
    
    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* _fdata){

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

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<DJI::OSDK::WayPointSettings> BackendDJI::createWaypoints(std::vector<Eigen::Vector3f> _wayPoints, dataMission _config){
            
        // Let's create a vector to store our waypoints in
        std::vector<DJI::OSDK::WayPointSettings> wp_list;

        for (unsigned i = 0; i < _wayPoints.size(); i++){

            DJI::OSDK::WayPointSettings  wp;
            setWaypointDefaults(&wp);

            wp.turnMode = _config.turnModeWP;
            wp.index     = i;
            wp.latitude  = _wayPoints[i](0);
            wp.longitude = _wayPoints[i](1);
            wp.altitude  = _wayPoints[i](2);
            wp_list.push_back(wp);

        }

        // Come back home
        DJI::OSDK::WayPointSettings final_wp;
        setWaypointDefaults(&final_wp);

        final_wp.turnMode = _config.turnModeWP;
        final_wp.index = _wayPoints.size();
        final_wp.latitude  = _wayPoints[0](0);
        final_wp.longitude = _wayPoints[0](1);
        final_wp.altitude  = _wayPoints[0](2);

        wp_list.push_back(final_wp);        

        return wp_list;

    }

    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& _wpList){

        for (std::vector<DJI::OSDK::WayPointSettings>::iterator wp = _wpList.begin(); wp != _wpList.end(); ++wp){
            if(useLogStatus_){
                LogStatus::get()->status("Waypoint created at (LLA): " + std::to_string(wp->latitude) + " | " + std::to_string(wp->longitude) + " | " + std::to_string(wp->altitude), true);
            }
            secureGuard_.lock();
            DJI::OSDK::ACK::WayPointIndex wpDataACK = vehicle_->missionManager->wpMission->uploadIndexData(&(*wp), functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(wpDataACK.ack)){
                DJI::OSDK::ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
            }
            
        }

    }

    //-----------------------------------------------------------------------------------------------------------------
    DJI::OSDK::Telemetry::Vector3f BackendDJI::toEulerAngle(void* _quaternionData){
    
        DJI::OSDK::Telemetry::Vector3f    result;
        DJI::OSDK::Telemetry::Quaternion* quaternion = (DJI::OSDK::Telemetry::Quaternion*)_quaternionData;

        double q2sqr = quaternion->q2 * quaternion->q2;
        double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
        double t1 = +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
        double t2 = -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
        double t3 = +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
        double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

        t2 = (t2 > 1.0) ? 1.0 : t2;
        t2 = (t2 < -1.0) ? -1.0 : t2;

        result.x = asin(t2);
        result.y = atan2(t3, t4);
        result.z = atan2(t1, t0);

        return result;
    
    }

}
