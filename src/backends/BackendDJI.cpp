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


#include <dal/backends/BackendDJI.h>

namespace dal{
    //-----------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::takeOff(const float _height){

        // Start takeoff
        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode takeoffStatus = vehicle_->control->takeoff(functionTimeout_);
        secureGuard_.unlock();
        if(DJI::OSDK::ACK::getError(takeoffStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(takeoffStatus, __func__);
            LogStatus::get()->error("Error at start takeoff, exiting", true);
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
            LogStatus::get()->error("Takeoff failed. Motors are not spinning, exiting", true);
            // Cleanup
            unsubscribeToData();     // 666 TODO: Make this better  
            return false; 
        }else{
            LogStatus::get()->status("Motors spinning...", true);     
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
            LogStatus::get()->error("Takeoff failed. Aircraft is still on the ground, but the motors are spinning, exiting", true);
            // Cleanup
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }else{
            LogStatus::get()->status("Ascending...", true);
        }

        // Final check: Finished takeoff
        while(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF){
                
                sleep(1);
        }

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
            
                LogStatus::get()->status("Successful takeoff!", true);
        }else{
            LogStatus::get()->error("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
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
            LogStatus::get()->error("Error at land, exiting", true);
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
            LogStatus::get()->error("Landing failed. Aircraft is still in the air, exiting", true);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }else{
            LogStatus::get()->status("Landing...", true);
        }

        // Second check: Finished landing
        while (vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR){
                
                sleep(1);
        }

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
                
                LogStatus::get()->status("Successful landing!", true);
        }else{
            LogStatus::get()->error("Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }
        
        return true;        
    }

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
    bool BackendDJI::mission(std::vector<Eigen::Vector3f> _wayPoints, std::string _missionType){

        if(_missionType == "waypoint"){

            // WAYPOINTS MISSION
            missionType_ = "waypoint";

            // Waypoint Mission : Initialization
            DJI::OSDK::WayPointInitSettings fdata;
            setWaypointInitDefaults(&fdata);

            int numWaypoints = _wayPoints.size();
            fdata.indexNumber = numWaypoints + 1; // We add 1 to get the aircarft back to the start

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode initAck = vehicle_->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT, functionTimeout_, &fdata);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(initAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
                LogStatus::get()->error("Error at init mission manager, exiting", true);
                // unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }

            secureGuard_.lock();
            vehicle_->missionManager->printInfo();
            secureGuard_.unlock();
            LogStatus::get()->status("Initializing Waypoint Mission...", true);

            // Waypoint Mission: Create Waypoints
            std::vector<WayPointSettings> generatedWaypts = createWaypoints(_wayPoints);
            LogStatus::get()->status("Creating Waypoints...", true);

            // Waypoint Mission: Upload the waypoints
            uploadWaypoints(generatedWaypts);
            LogStatus::get()->status("Uploading Waypoints...", true);

        }else if(_missionType == "hotpoint"){

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
            vehicle_->missionManager->hpMission->setHotPoint(_wayPoints[0](0), _wayPoints[0](1), _wayPoints[0](2));
            secureGuard_.unlock();

        }else{
            LogStatus::get()->error("Unrecognised mission type, exiting", true);
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
                LogStatus::get()->error("Error at start mission, exiting", true);
                // unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Starting Waypoint Mission...", true);
            }

        }else if(missionType_ == "hotpoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode startAck = vehicle_->missionManager->hpMission->start(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(startAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
                LogStatus::get()->error("Error at start mission, exiting", true);
                // unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Starting Waypoint Mission...", true);
            }

        }else{
            LogStatus::get()->error("Unrecognised mission type, exiting", true);
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
                LogStatus::get()->error("Error at pause mission, exiting", true);
                unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Pause Waypoint Mission...", true);
            }

        }else if(missionType_ == "hotpoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode pauseAck = vehicle_->missionManager->hpMission->pause(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(pauseAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(pauseAck, __func__);
                LogStatus::get()->error("Error at pause mission, exiting", true);
                unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Pause Waypoint Mission...", true);
            }

        }else{
            LogStatus::get()->error("Unrecognised mission type, exiting", true);
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
                LogStatus::get()->error("Error at stop mission, exiting", true);
                unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Stop Waypoint Mission...", true);
            }

        }else if(missionType_ == "hotpoint"){
            
            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode stopAck = vehicle_->missionManager->hpMission->stop(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(stopAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(stopAck, __func__);
                LogStatus::get()->error("Error at stop mission, exiting", true);
                unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Stop Waypoint Mission...", true);
            }

        }else{
            LogStatus::get()->error("Unrecognised mission type, exiting", true);
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
                LogStatus::get()->error("Error at resume mission, exiting", true);
                unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Resume Waypoint Mission...", true);
            }

        }else if(missionType_ == "hotpoint"){

            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode resumeAck = vehicle_->missionManager->hpMission->resume(functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(resumeAck)){
                DJI::OSDK::ACK::getErrorCodeMessage(resumeAck, __func__);
                LogStatus::get()->error("Error at resume mission, exiting", true);
                unsubscribeToData();     // 666 TODO: Make this better  
                return false;
            }else{
                LogStatus::get()->status("Resume Waypoint Mission...", true);
            }

        }else{
            LogStatus::get()->error("Unrecognised mission type, exiting", true);
            return false;
        }

        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::positionCtrlYaw(float _x, float _y, float _z, float _yaw){

        if(vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            return false;
        }

        LogStatus::get()->status("Moving in local position", false);

        // Convert position offset from first position to local coordinates
        DJI::OSDK::Telemetry::Vector3f localOffset;

        secureGuard_.lock();
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        secureGuard_.unlock();

        localOffsetFromGpsOffset(localOffset, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&originGPS_));

        // Get initial offset. We will update this in a loop later.
        double xOffset = _x - localOffset.x;
        double yOffset = _y - localOffset.y;
        double zOffset = _z - localOffset.z;

        // 0.1 m or 10 cms is the minimum error to reach target in x, y and z axes.
        // This error threshold will have to change depending on aircraft / payload / wind conditions
        double xCmd, yCmd, zCmd;
        if(((std::abs(xOffset)) < 0.1) && ((std::abs(yOffset)) < 0.1) && (localOffset.z > (zOffset - 0.1)) && (localOffset.z < (zOffset + 0.1))){
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
        vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            int mode = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            return false;
        }
        
        LogStatus::get()->status("Moving in velocity", false);
        
        secureGuard_.lock();
        vehicle_->control->velocityAndYawRateCtrl(_vx, _vy, _vz, _yawRate);
        secureGuard_.unlock();
        
        return true;
    }    

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::receiveTelemetry(dataTelemetry& _data, bool _saveToFile){
        
        // Get all the data once before the loop to initialize vars
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>::type          flightStatus;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>::type     mode;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type              latLon;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type      altitude;
        DJI::OSDK::Telemetry::Vector3f                                                          localOffset;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC>::type                     rc;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_VELOCITY>::type               velocity;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type             quaternion;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION>::type           rtk;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO>::type      rtk_pos_info;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY>::type           rtk_velocity;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_YAW>::type                rtk_yaw;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO>::type           rtk_yaw_info;

        secureGuard_.lock();
        flightStatus = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        secureGuard_.unlock();

        secureGuard_.lock();
        mode        = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        secureGuard_.unlock();

        secureGuard_.lock();
        latLon       = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        secureGuard_.unlock();

        secureGuard_.lock();
        altitude     = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
        secureGuard_.unlock();

        localOffsetFromGpsOffset(localOffset, static_cast<void*>(&latLon), static_cast<void*>(&originGPS_));

        secureGuard_.lock();
        rc           = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC>();
        secureGuard_.unlock();

        secureGuard_.lock();
        velocity     = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
        secureGuard_.unlock();

        secureGuard_.lock();
        quaternion   = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();
        secureGuard_.unlock();

        std::string sFlightStatus, sMode;
        if(flightStatus == 0){
            sFlightStatus = "STOPED";
        }else if(flightStatus == 1){
            sFlightStatus = "ON_GROUND";
        }else if(flightStatus == 2){
            sFlightStatus = "IN_AIR";
        }else{
            sFlightStatus = "UNRECOGNIZED";
        }

        if(mode == 0){
            sMode = "MANUAL";
        }else if(mode == 1){
            sMode = "ATTITUDE";
        }else if(mode == 6){
            sMode = "P_GPS";
        }else if(mode == 9){
            sMode = "HOT_POINT";
        }else if(mode == 10){
            sMode = "ASSISTED_TAKEOFF";
        }else if(mode == 11){
            sMode = "AUTO_TAKEOFF";
        }else if(mode == 12){
            sMode = "ASSISTED_LAND";
        }else if(mode == 15){
            sMode = "GO_HOME";
        }else if(mode == 17){
            sMode = "NAVI_SDK_CTMODE_CTRLRL";
        }else if(mode == 33){
            sMode = "FORCE_AUTO_LANDING";
        }else if(mode == 40){
            sMode = "SEARCH";
        }else if(mode == 41){
            sMode = "ENGINE_START";
        }else{
            sMode = "UNRECOGNIZED";
        }

        _data.flightStatus = sFlightStatus; 
        _data.mode = sMode;
        _data.latLon(0) = latLon.latitude;
        _data.latLon(1) = latLon.longitude; 
        _data.nGPS = latLon.visibleSatelliteNumber; 
        _data.altitude = altitude; 
        _data.localPosition(0) = localOffset.x;
        _data.localPosition(1) = localOffset.y;
        _data.localPosition(2) = localOffset.z;
        _data.rc(0) = rc.roll; 
        _data.rc(1) = rc.pitch; 
        _data.rc(2) = rc.yaw; 
        _data.rc(3) = rc.throttle; 
        _data.velocity(0) = velocity.data.x; 
        _data.velocity(1) = velocity.data.y; 
        _data.velocity(2) = velocity.data.z; 
        _data.quaternion(0) = quaternion.q0; 
        _data.quaternion(1) = quaternion.q1; 
        _data.quaternion(2) = quaternion.q2; 
        _data.quaternion(3) = quaternion.q3; 

        if(rtkAvailable_){
            secureGuard_.lock();
            rtk = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION>();
            secureGuard_.unlock();

            secureGuard_.lock();
            rtk_pos_info = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO>();
            secureGuard_.unlock();
            
            secureGuard_.lock();
            rtk_velocity = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY>();
            secureGuard_.unlock();
            
            secureGuard_.lock();
            rtk_yaw = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_YAW>();
            secureGuard_.unlock();
            
            secureGuard_.lock();
            rtk_yaw_info = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO>();
            secureGuard_.unlock();

            _data.rtk(0) = rtk.latitude; 
            _data.rtk(1) = rtk.longitude; 
            _data.rtk(2) = rtk.HFSL; 
            _data.rtk(3) = rtk_velocity.x; 
            _data.rtk(4) = rtk_velocity.y; 
            _data.rtk(5) = rtk_velocity.z; 
            _data.rtk(6) = rtk_yaw; 
            _data.rtk(7) = rtk_yaw_info; 
            _data.rtk(8) = rtk_pos_info; 
        }else{
            _data.rtk(0) = 0; 
            _data.rtk(1) = 0; 
            _data.rtk(2) = 0; 
            _data.rtk(3) = 0; 
            _data.rtk(4) = 0; 
            _data.rtk(5) = 0; 
            _data.rtk(6) = 0;
            _data.rtk(7) = 0; 
            _data.rtk(8) = 0; 
        }

        if(_saveToFile){
            // Flight Status / Mode / Position Latitude / Position Longitude / Position Altitude / Satellite number / Local Position x / Local Position y / Local Position z / Roll / Pitch / Yaw / Throttle / Vx / Vy / Vz / Qw / Qx / Qy / Qz / RTK Latitude / RTK Longitude / RTK Altitude / RTK Vx / RTK Vy / RTK Vz / RTK Yaw / RTK Yaw info / RTK Position info
            std::string stringGeneral = std::to_string((int)flightStatus) + " " + sMode + " " + std::to_string(latLon.latitude) + " " + std::to_string(latLon.longitude) + " " + std::to_string(altitude) + " " + std::to_string((int)latLon.visibleSatelliteNumber) + " " + std::to_string(localOffset.x) + " " + std::to_string(localOffset.y) + " " + std::to_string(localOffset.z) + " " + std::to_string(rc.roll) + " " + std::to_string(rc.pitch) + " " + std::to_string(rc.yaw) + " " + std::to_string(rc.throttle) + " " + std::to_string(velocity.data.x) + " " + std::to_string(velocity.data.y) + " " + std::to_string(velocity.data.z) + " " + std::to_string(quaternion.q0) + " " + std::to_string(quaternion.q1) + " " + std::to_string(quaternion.q2) + " " + std::to_string(quaternion.q3);
            std::string stringRTK = std::to_string(rtk.latitude) + " " + std::to_string(rtk.longitude) + " " + std::to_string(rtk.HFSL) + " " + std::to_string(rtk_velocity.x) + " " + std::to_string(rtk_velocity.y) + " " + std::to_string(rtk_velocity.z) + " " + std::to_string(rtk_yaw) + " " + std::to_string(rtk_yaw_info) + " " + std::to_string(rtk_pos_info);
            std::string stringTelemetry = stringGeneral + " " + stringRTK;
            LogTelemetry::get()->message(stringTelemetry, false);
        }
                    
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::init(const Config &_config){

        LogStatus::init("DJIStatus_" + std::to_string(time(NULL)));
        LogTelemetry::init("DJITelemetry_" + std::to_string(time(NULL)));

        // Setup Vehicle
        secureGuard_.lock();
        vehicle_ = new DJI::OSDK::Vehicle(_config.device.c_str(),
                                   _config.baudrate,
                                   true,
                                   _config.useAdvancedSensing);

        // Check if the communication is working fine
        if (!vehicle_->protocolLayer->getDriver()->getDeviceStatus())
        {
            LogStatus::get()->error("Comms appear to be incorrectly set up exiting", true);
            return false;
        }
        secureGuard_.unlock();

        // Activate
        activateData_.ID = _config.app_id;
        char app_key[65];
        activateData_.encKey = app_key;
        std::strcpy(activateData_.encKey, _config.app_key.c_str());

        secureGuard_.lock();
        activateData_.version = vehicle_->getFwVersion();
        DJI::OSDK::ACK::ErrorCode ack = vehicle_->activate(&activateData_, functionTimeout_);
        secureGuard_.unlock();

        if (DJI::OSDK::ACK::getError(ack))
        {
            DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
            LogStatus::get()->error("Error validating Vehicle, exiting", true);
            return false;
        }

        if(vehicle_ == NULL){
            LogStatus::get()->error("Vehicle not initialized, exiting", true);
            return false;
        }
        LogStatus::get()->status("Vehicle initialized", true);
      
        // Obtain Control Authority
        if(obtainControlAuthority(true)){
            if(subscribeToData()){     // 666 TODO: MAKE THIS BETTER, UNA ESPECIE DE SINGLETON?
            LogStatus::get()->status("Subscribe To Data success", true);
                return true;
            }else{
                LogStatus::get()->status("Subscribe To Data failed", true);
                return false;
            }
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
            LogStatus::get()->error("Not obtained Control Authority, exiting", _info);
            return false;
        }
        LogStatus::get()->status("Obtained Control Authority of the Vehicle", _info);
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::subscribeToData(){     // 666 TODO: MAKE THIS BETTER, UNA ESPECIE DE SINGLETON?

        // We will subscribe to six kinds of data:
        // 1. Flight Status at 10 Hz
        // 2. Mode at 10 Hz
        // 3. Fused Lat/Lon at 50Hz
        // 4. Fused Altitude at 50Hz
        // 5. RC Channels at 50 Hz
        // 6. Velocity at 50 Hz
        // 7. Quaternion at 200 Hz
        // 8. RTK if available at 5 Hz

        // Please make sure your drone is in simulation mode. You can fly the drone with your RC to get different values.
	
        // Telemetry: Verify the subscription
        DJI::OSDK::ACK::ErrorCode subscribeStatus;
        
        secureGuard_.lock();
        subscribeStatus = vehicle_->subscribe->verify(functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            LogStatus::get()->error("Not verified subscription to Telemetry, exiting", true);
            return false;
        }
	
        // Package 0: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex_ = 0;
        int freq = 10;
        DJI::OSDK::Telemetry::TopicName topicList10Hz[] = { DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT, DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE};
        int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        secureGuard_.lock();
        bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList10Hz, enableTimestamp, freq);
        secureGuard_.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 0 from topic list, exiting", true);
            return false;
        }

        secureGuard_.lock();
        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            LogStatus::get()->error("Start package 0 error, exiting", true);
            return false;
        }

        // Package 1: Subscribe to Lat/Lon, Alt, RC Channel and Velocity at freq 50 Hz
        pkgIndex_ = 1;
        freq = 50;
        DJI::OSDK::Telemetry::TopicName topicList50Hz[] = { DJI::OSDK::Telemetry::TOPIC_GPS_FUSED, DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED, DJI::OSDK::Telemetry::TOPIC_RC, DJI::OSDK::Telemetry::TOPIC_VELOCITY};
        numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
        enableTimestamp = false;

        secureGuard_.lock();
        pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList50Hz, enableTimestamp, freq);
        secureGuard_.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 1 from topic list, exiting", true);
            return false;
        }

        secureGuard_.lock();
        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            LogStatus::get()->error("Start package 1 error, exiting", true);
            return false;
        }

        // Package 2: Subscribe to Quaternion at freq 200 Hz.
        pkgIndex_ = 2;
        freq = 200;
        DJI::OSDK::Telemetry::TopicName topicList200Hz[] = { DJI::OSDK::Telemetry::TOPIC_QUATERNION };
        numTopic = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
        enableTimestamp = false;

        secureGuard_.lock();
        pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList200Hz, enableTimestamp, freq);
        secureGuard_.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 2 from topic list, exiting", true);
            return false;
        }

        secureGuard_.lock();
        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
        secureGuard_.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            LogStatus::get()->error("Start package 2 error, exiting", true);
            return false;
        }

        // Package 3: Subscribe to RTK at freq 5 Hz.
        pkgIndex_ = 3;
        freq = 5;
        DJI::OSDK::Telemetry::TopicName topicListRTK5Hz[] = {DJI::OSDK::Telemetry::TOPIC_RTK_POSITION, DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO, DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO, DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY, DJI::OSDK::Telemetry::TOPIC_RTK_YAW};
        numTopic = sizeof(topicListRTK5Hz) / sizeof(topicListRTK5Hz[0]);
        enableTimestamp = false;

        secureGuard_.lock();
        pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicListRTK5Hz, enableTimestamp, freq);
        secureGuard_.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 3 from topic list, exiting", true);
            return false;
        }else{
            secureGuard_.lock();
            subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
            secureGuard_.unlock();
            if(subscribeStatus.data == DJI::OSDK::ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE){
                LogStatus::get()->status("RTK Not Available", true);
                rtkAvailable_ = false;
            }else{
                rtkAvailable_ = true;
                LogStatus::get()->status("RTK Available", true);
                if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS) {
                    DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
                    // Cleanup before return
                    unsubscribeToData();     // 666 TODO: Make this better  
                    LogStatus::get()->error("Start package 3 error, exiting", true);
                    return false;
                }
            }
        }

        // Wait for the data to start coming in.
        sleep(1);

        // Also, since we don't have a source for relative height through subscription,
        // start using broadcast height
        if (!startGlobalPositionBroadcast()){
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better 
            return false;
        }

        // Wait for the data to start coming in.
        sleep(1);
        
        return setLocalPosition();

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::unsubscribeToData(){

        for(int i = 0; i < pkgIndex_; i++ ){
            secureGuard_.lock();
            DJI::OSDK::ACK::ErrorCode ack = vehicle_->subscribe->removePackage(i, functionTimeout_);
            secureGuard_.unlock();
            if (DJI::OSDK::ACK::getError(ack)){
                LogStatus::get()->error("Error unsubscribing at package: " + std::to_string(i) + "; please restart the drone/FC to get back to a clean state, exiting", true);
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
        
        // Get the broadcast GP since we need the height
        secureGuard_.lock();
        broadcastGP_ = vehicle_->broadcast->getGlobalPosition();
        secureGuard_.unlock();

        return true;

    }
    
    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::localOffsetFromGpsOffset(DJI::OSDK::Telemetry::Vector3f& _delta, void* _target, void* _origin){
    
        DJI::OSDK::Telemetry::GPSFused* subscriptionTarget = (DJI::OSDK::Telemetry::GPSFused*)_target;
        DJI::OSDK::Telemetry::GPSFused*  subscriptionOrigin = (DJI::OSDK::Telemetry::GPSFused*)_origin;
        
        double t_lon = subscriptionTarget->longitude * 180.0 / C_PI;
        double r_lon = subscriptionOrigin->longitude * 180.0 / C_PI;
        
        double t_lat = subscriptionTarget->latitude * 180.0 / C_PI;
        double r_lat = subscriptionOrigin->latitude * 180.0 / C_PI;

        double deltaLon   = t_lon - r_lon;
        double deltaLat   = t_lat - r_lat;

        // NED ? NEED TO CHECK
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
        for (int i = 0; i < 16; ++i)
        {
            _wp->commandList[i]      = 0;
            _wp->commandParameter[i] = 0;
        }

    }
    
    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* _wp){

        _wp->maxVelocity    = 10;
        _wp->idleVelocity   = 5;
        _wp->finishAction   = 0;
        _wp->executiveTimes = 1;
        _wp->yawMode        = 0;
        _wp->traceMode      = 0;
        _wp->RCLostAction   = 1;
        _wp->gimbalPitch    = 0;
        _wp->latitude       = 0;
        _wp->longitude      = 0;
        _wp->altitude       = 0;

    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<DJI::OSDK::WayPointSettings> BackendDJI::createWaypoints(std::vector<Eigen::Vector3f> _wayPoints){
            
        // Let's create a vector to store our waypoints in
        std::vector<DJI::OSDK::WayPointSettings> wp_list;

        for (int i = 0; i < _wayPoints.size(); i++){

            DJI::OSDK::WayPointSettings  wp;
            setWaypointDefaults(&wp);
            wp.index     = i;
            wp.latitude  = _wayPoints[i](0);
            wp.longitude = _wayPoints[i](1);
            wp.altitude  = _wayPoints[i](2);
            wp_list.push_back(wp);

        }

        // Come back home
        DJI::OSDK::WayPointSettings final_wp;
        setWaypointDefaults(&final_wp);

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
            
            printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
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
        DJI::OSDK::Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)_quaternionData;

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

    bool BackendDJI::startGlobalPositionBroadcast(){
        uint8_t freq[16];

        /* Channels definition for A3/N3/M600
        * 0 - Timestamp
        * 1 - Attitude Quaternions
        * 2 - Acceleration
        * 3 - Velocity (Ground Frame)
        * 4 - Angular Velocity (Body Frame)
        * 5 - Position
        * 6 - GPS Detailed Information
        * 7 - RTK Detailed Information
        * 8 - Magnetometer
        * 9 - RC Channels Data
        * 10 - Gimbal Data
        * 11 - Flight Status
        * 12 - Battery Level
        * 13 - Control Information
        */
        freq[0]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[1]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[2]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[3]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[4]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[5]  =  DJI::OSDK::DataBroadcast::FREQ_50HZ; // This is the only one we want to change
        freq[6]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[7]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[8]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[9]  =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[10] =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[11] =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[12] =  DJI::OSDK::DataBroadcast::FREQ_HOLD;
        freq[13] =  DJI::OSDK::DataBroadcast::FREQ_HOLD;

        secureGuard_.lock();
        DJI::OSDK::ACK::ErrorCode ack = vehicle_->broadcast->setBroadcastFreq(freq, 1);
        secureGuard_.unlock();
        if(DJI::OSDK::ACK::getError(ack)){
            DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
            LogStatus::get()->error("Global Position Broadcast error, exiting", true);
            return false;
        }
        else{
            return true;
        }
    }

}
