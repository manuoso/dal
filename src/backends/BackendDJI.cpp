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
        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode takeoffStatus = mVehicle->control->takeoff(mFunctionTimeout);
        mSecureGuard.unlock();
        if(DJI::OSDK::ACK::getError(takeoffStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(takeoffStatus, __func__);
            LogStatus::get()->error("Error at start takeoff, exiting", true);
            return false;
        }

        // First check: Motors started
        int motorsNotStarted = 0;
        int timeoutCycles    = 20;

        while(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() != DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND &&
            mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START &&
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

        while(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR &&
            (mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
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
        while(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF){
                
                sleep(1);
        }

        if(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
            
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
        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode landingStatus = mVehicle->control->land(mFunctionTimeout);
        mSecureGuard.unlock();
        if(DJI::OSDK::ACK::getError(landingStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(landingStatus, __func__);
            LogStatus::get()->error("Error at land, exiting", true);
            return false;
        }

        // First check: Landing started
        int landingNotStarted = 0;
        int timeoutCycles     = 20;

        while (mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
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
        while (mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR){
                
                sleep(1);
        }

        if(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
                
                LogStatus::get()->status("Successful landing!", true);
        }else{
            LogStatus::get()->error("Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }
        
        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::movePosition(float _x, float _y, float _z, float _yaw, float _posThreshold, float _yawThreshold){
        
        int timeoutInMilSec = 10000;
        int controlFreqInHz = 50; // Hz
        int cycleTimeInMs = 1000 / controlFreqInHz;
        int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs; // 10 cycles
        int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles

        // Convert position offset from first position to local coordinates
         DJI::OSDK::Telemetry::Vector3f localOffsetNed, localOffsetEnu;

        mSecureGuard.lock();
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        mSecureGuard.unlock();

        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type originSubscriptionGPS  = currentSubscriptionGPS;
        localOffsetFromGpsOffset(localOffsetNed, localOffsetEnu, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&originSubscriptionGPS));

        // Get the broadcast GP since we need the height for zCmd
        mSecureGuard.lock();
        DJI::OSDK::Telemetry::GlobalPosition currentBroadcastGP = mVehicle->broadcast->getGlobalPosition();
        mSecureGuard.unlock();

        // Get initial offset. We will update this in a loop later.
        double xOffsetRemaining = _x - localOffsetNed.x;
        double yOffsetRemaining = _y - localOffsetNed.y;
        double zOffsetRemaining = _z - (-localOffsetNed.z);

        // Conversions
        double _yawRad     = DEG2RAD * _yaw;
        double yawThresholdInRad = DEG2RAD * _yawThreshold;

        //! Get Euler angle
        mSecureGuard.lock();
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type subscriptionQ = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();
        mSecureGuard.unlock();
        double yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;

        int elapsedTimeInMs = 0;
        int withinBoundsCounter = 0;
        int outOfBounds = 0;
        int brakeCounter = 0;
        int speedFactor = 2;
        float xCmd, yCmd, zCmd;

        // There is a deadband in position control
        // the z cmd is absolute height
        // while x and y are in relative
        float zDeadband = 0.12;        

        /*! Calculate the inputs to send the position controller. We implement basic
        *  receding setpoint position control and the setpoint is always 1 m away
        *  from the current position - until we get within a threshold of the goal.
        *  From that point on, we send the remaining distance as the setpoint.
        */
        if (_x > 0){
            xCmd = (_x < speedFactor) ? _x : speedFactor;
        }else if (_x < 0){
            xCmd = (_x > -1 * speedFactor) ? _x : -1 * speedFactor;
        }else{
            xCmd = 0;
        }   

        if (_y > 0){
            yCmd = (_y < speedFactor) ? _y : speedFactor;
        }else if (_y < 0){
            yCmd = (_y > -1 * speedFactor) ? _y : -1 * speedFactor;
        }else{
            yCmd = 0;
        }
        
        zCmd = currentBroadcastGP.height + _z; //Since subscription cannot give us a relative height, use broadcast.
        
        LogStatus::get()->status("(Before while) xCmd: " + std::to_string(xCmd) + " yCmd: " + std::to_string(yCmd) + " zCmd: " + std::to_string(zCmd), true);
        LogStatus::get()->status("(Before while) xOffsetRemaining: " + std::to_string(xOffsetRemaining) + " yOffsetRemaining: " + std::to_string(yOffsetRemaining) + " zOffsetRemaining: " + std::to_string(zOffsetRemaining), true);

        //! Main closed-loop receding setpoint position control
        while (elapsedTimeInMs < timeoutInMilSec){

            LogStatus::get()->status("(While) xCmd: " + std::to_string(xCmd) + " yCmd: " + std::to_string(yCmd) + " zCmd: " + std::to_string(zCmd), true);
            LogStatus::get()->status("(While) xOffsetRemaining: " + std::to_string(xOffsetRemaining) + " yOffsetRemaining: " + std::to_string(yOffsetRemaining) + " zOffsetRemaining: " + std::to_string(zOffsetRemaining), true);

            mSecureGuard.lock();
            mVehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd, _yawRad / DEG2RAD);
            mSecureGuard.unlock();

            usleep(cycleTimeInMs * 1000);
            elapsedTimeInMs += cycleTimeInMs;

            //! Get current position in required coordinates and units
            mSecureGuard.lock();
            subscriptionQ = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();
            mSecureGuard.unlock();
            yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;

            mSecureGuard.lock();
            currentSubscriptionGPS = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
            mSecureGuard.unlock();
            localOffsetFromGpsOffset(localOffsetNed, localOffsetEnu, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&originSubscriptionGPS));

            // Get the broadcast GP since we need the height for zCmd
            mSecureGuard.lock();
            currentBroadcastGP = mVehicle->broadcast->getGlobalPosition();
            mSecureGuard.unlock();

            //! See how much farther we have to go
            xOffsetRemaining = _x - localOffsetNed.x;
            yOffsetRemaining = _y - localOffsetNed.y;
            zOffsetRemaining = _z - (-localOffsetNed.z);

            //! See if we need to modify the setpoint
            if (std::abs(xOffsetRemaining) < speedFactor){
                xCmd = xOffsetRemaining;
            }
            if (std::abs(yOffsetRemaining) < speedFactor){
                yCmd = yOffsetRemaining;
            }

            if (std::abs(xOffsetRemaining) < _posThreshold &&
                    std::abs(yOffsetRemaining) < _posThreshold &&
                    std::abs(zOffsetRemaining) < zDeadband &&
                    std::abs(yawInRad - _yawRad) < yawThresholdInRad){
                    
                    //! 1. We are within bounds; start incrementing our in-bound counter
                    withinBoundsCounter += cycleTimeInMs;
            }else{
                if (withinBoundsCounter != 0){
                    //! 2. Start incrementing an out-of-bounds counter
                    outOfBounds += cycleTimeInMs;
                }
            }
            //! 3. Reset withinBoundsCounter if necessary
            if (outOfBounds > outOfControlBoundsTimeLimit){
                withinBoundsCounter = 0;
                outOfBounds = 0;
            }
            //! 4. If within bounds, set flag and break
            if (withinBoundsCounter >= withinControlBoundsTimeReqmt){
                break;
            }
        }

        //! Set velocity to zero, to prevent any residual velocity from position
        //! command
        while (brakeCounter < withinControlBoundsTimeReqmt){
            mSecureGuard.lock();
            mVehicle->control->emergencyBrake();
            mSecureGuard.unlock();
            usleep(cycleTimeInMs * 10);
            brakeCounter += cycleTimeInMs;
        }

        if (elapsedTimeInMs >= timeoutInMilSec){
            std::cout << "Task timeout!\n";
            // unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }
                
        return true;
       
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::positionCtrlYaw(float _x, float _y, float _z, float _yaw, bool _offset){
        
        if(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            int mode = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            return false;
        }

        if(_offset){
            LogStatus::get()->status("Moving in global local position", true);

            // Convert position offset from first position to local coordinates
            DJI::OSDK::Telemetry::Vector3f localOffsetNed, localOffsetEnu;

            mSecureGuard.lock();
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
            mSecureGuard.unlock();

            localOffsetFromGpsOffset(localOffsetNed, localOffsetEnu, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&mOriginGPS));

            // Get the broadcast GP since we need the height for zCmd
            mSecureGuard.lock();
            DJI::OSDK::Telemetry::GlobalPosition currentBroadcastGP = mVehicle->broadcast->getGlobalPosition();
            mSecureGuard.unlock();

            // Get initial offset. We will update this in a loop later.
            double xOffset = _x - localOffsetNed.x;
            double yOffset = _y - localOffsetNed.y;
            double zOffset = _z - (localOffsetNed.z);

            // double zOffset = currentBroadcastGP.height + _z; //Since subscription cannot give us a relative height, use broadcast
 
            mSecureGuard.lock();
            mVehicle->control->positionAndYawCtrl(xOffset, yOffset, zOffset, _yaw);
            mSecureGuard.unlock();

        }else{
            LogStatus::get()->status("Moving in local position", true);

            // Get the broadcast GP since we need the height for z
            mSecureGuard.lock();
            DJI::OSDK::Telemetry::GlobalPosition currentBroadcastGP = mVehicle->broadcast->getGlobalPosition();
            mSecureGuard.unlock();

            double zOffset = currentBroadcastGP.height + _z;

            mSecureGuard.lock();
            mVehicle->control->positionAndYawCtrl(_x, _y, zOffset, _yaw);
            mSecureGuard.unlock();

        }

        return true;
    }
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate){

        // _vx: velocity in x axis (m/s) 
        // _vy: velocity in y axis (m/s) 
        // _vz: velocity in z axis (m/s) 
        // _yawRate: yawRate set-point (deg/s) 

        if(mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            int mode = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            LogStatus::get()->error("Error vehicle not in mode to control with API SDK, mode: " + std::to_string(mode) + " exiting", true);
            return false;
        }
        
        LogStatus::get()->status("Moving in velocity", true);
        
        mSecureGuard.lock();
        mVehicle->control->velocityAndYawRateCtrl(_vx, _vy, _vz, _yawRate);
        mSecureGuard.unlock();
        
        return true;
    }    

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::receiveTelemetry(dataTelemetry& _data, bool _printData, bool _saveToFile){
        
        // Get all the data once before the loop to initialize vars
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>::type          flightStatus;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>::type     mode;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type              latLon;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type      altitude;
        DJI::OSDK::Telemetry::Vector3f                                                          localOffsetNed, localOffsetEnu;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC>::type                     rc;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_VELOCITY>::type               velocity;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type             quaternion;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION>::type           rtk;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO>::type      rtk_pos_info;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY>::type           rtk_velocity;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_YAW>::type                rtk_yaw;
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO>::type           rtk_yaw_info;

        mSecureGuard.lock();
        flightStatus = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        mSecureGuard.unlock();

        mSecureGuard.lock();
        mode        = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        mSecureGuard.unlock();

        mSecureGuard.lock();
        latLon       = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        mSecureGuard.unlock();

        mSecureGuard.lock();
        altitude     = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
        mSecureGuard.unlock();

        localOffsetFromGpsOffset(localOffsetNed, localOffsetEnu, static_cast<void*>(&latLon), static_cast<void*>(&mOriginGPS));

        mSecureGuard.lock();
        rc           = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC>();
        mSecureGuard.unlock();

        mSecureGuard.lock();
        velocity     = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
        mSecureGuard.unlock();

        mSecureGuard.lock();
        quaternion   = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();
        mSecureGuard.unlock();

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
        _data.altitude = altitude; 
        _data.localPositionNED(0) = localOffsetNed.x;
        _data.localPositionNED(1) = localOffsetNed.y;
        _data.localPositionNED(2) = localOffsetNed.z;
        _data.localPositionENU(0) = localOffsetEnu.x;
        _data.localPositionENU(1) = localOffsetEnu.y;
        _data.localPositionENU(2) = localOffsetEnu.z;
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

        if(mRTKAvailable){
            mSecureGuard.lock();
            rtk = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION>();
            mSecureGuard.unlock();

            mSecureGuard.lock();
            rtk_pos_info = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO>();
            mSecureGuard.unlock();
            
            mSecureGuard.lock();
            rtk_velocity = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY>();
            mSecureGuard.unlock();
            
            mSecureGuard.lock();
            rtk_yaw = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_YAW>();
            mSecureGuard.unlock();
            
            mSecureGuard.lock();
            rtk_yaw_info = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO>();
            mSecureGuard.unlock();

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

        if(_printData){
            std::cout << "Flight Status                         = " << (int)flightStatus
                    << "\n";
            std::cout << "Position              (LLA)           = " << latLon.latitude
                    << ", " << latLon.longitude << ", " << altitude << "\n";
            std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
                    << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
            std::cout << "Velocity              (vx,vy,vz)      = " << velocity.data.x
                    << ", " << velocity.data.y << ", " << velocity.data.z << "\n";
            std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
                    << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
                    << quaternion.q3 << "\n";
            if(mRTKAvailable) {
            std::cout << "RTK if available      (lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info) ="
                        << rtk.latitude << "," << rtk.longitude << "," << rtk.HFSL << "," << rtk_velocity.x << ","
                        << rtk_velocity.y
                        << "," << rtk_velocity.z << "," << rtk_yaw << "," << rtk_yaw_info << rtk_pos_info << "\n";
            }
            std::cout << "-------\n\n";
        }

        if(_saveToFile){
            // Flight Status / Position Latitude / Position Longitude / Position Altitude / Roll / Pitch / Yaw / Throttle / Vx / Vy / Vz / Qw / Qx / Qy / Qz / RTK Latitude / RTK Longitude / RTK Altitude / RTK Vx / RTK Vy / RTK Vz / RTK Yaw / RTK Yaw info / RTK Position info
            std::string stringFSPosRCVelQuat = std::to_string((int)flightStatus) + " " + std::to_string(latLon.latitude) + " " + std::to_string(latLon.longitude) + " " + std::to_string(altitude) + " " + std::to_string(rc.roll) + " " + std::to_string(rc.pitch) + " " + std::to_string(rc.yaw) + " " + std::to_string(rc.throttle) + " " + std::to_string(velocity.data.x) + " " + std::to_string(velocity.data.y) + " " + std::to_string(velocity.data.z) + " " + std::to_string(quaternion.q0) + " " + std::to_string(quaternion.q1) + " " + std::to_string(quaternion.q2) + " " + std::to_string(quaternion.q3);
            std::string stringRTK = std::to_string(rtk.latitude) + " " + std::to_string(rtk.longitude) + " " + std::to_string(rtk.HFSL) + " " + std::to_string(rtk_velocity.x) + " " + std::to_string(rtk_velocity.y) + " " + std::to_string(rtk_velocity.z) + " " + std::to_string(rtk_yaw) + " " + std::to_string(rtk_yaw_info) + " " + std::to_string(rtk_pos_info);
            std::string stringTelemetry = stringFSPosRCVelQuat + " " + stringRTK;
            LogTelemetry::get()->message(stringTelemetry, false);
        }
                    
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::runWaypointMissionPolygon(uint8_t _numWaypoints, float64_t _increment, float64_t _startAlt){
        
        // _increment -> 0.000001
        // _startAlt -> 10

        // Waypoint Mission : Initialization
        DJI::OSDK::WayPointInitSettings fdata;
        setWaypointInitDefaults(&fdata);

        fdata.indexNumber = _numWaypoints + 1; // We add 1 to get the aircarft back to the start.

        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode initAck = mVehicle->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT, mFunctionTimeout, &fdata);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(initAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
            LogStatus::get()->error("Error at init mission manager, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }

        mSecureGuard.lock();
        mVehicle->missionManager->printInfo();
        mSecureGuard.unlock();
        LogStatus::get()->status("Initializing Waypoint Mission...", true);

        // Waypoint Mission: Create Waypoints
        std::vector<WayPointSettings> generatedWaypts = createWaypoints(_numWaypoints, _increment, _startAlt);
        LogStatus::get()->status("Creating Waypoints...", true);

        // Waypoint Mission: Upload the waypoints
        uploadWaypoints(generatedWaypts);
        LogStatus::get()->status("Uploading Waypoints...", true);

        // Waypoint Mission: Start
        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode startAck = mVehicle->missionManager->wpMission->start(mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(startAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(initAck, __func__);
            LogStatus::get()->error("Error at start mission, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }else{
            LogStatus::get()->status("Starting Waypoint Mission...", true);
        }

        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::runHotpointMissionRadius(int _initialRadius, int _time){

        // Hotpoint Mission Initialize
        mSecureGuard.lock();
        mVehicle->missionManager->init(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT, mFunctionTimeout, NULL);
        mSecureGuard.unlock();

        mSecureGuard.lock();
        mVehicle->missionManager->printInfo();
        mSecureGuard.unlock();

        // Global position retrieved via subscription
        mSecureGuard.lock();
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type subscribeGPosition = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        mSecureGuard.unlock();

        mSecureGuard.lock();
        mVehicle->missionManager->hpMission->setHotPoint(subscribeGPosition.longitude, subscribeGPosition.latitude, _initialRadius);
        mSecureGuard.unlock();

        // Start
        LogStatus::get()->status("Start with default rotation rate: 15 deg/s", true);
        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode startAck = mVehicle->missionManager->hpMission->start(mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(startAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(startAck, __func__);
            LogStatus::get()->error("Error at start mission, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
        }

        sleep(_time);

        // // Pause
        // LogStatus::get()->status("Pause for 5s", true);
        // mSecureGuard.lock();
        // DJI::OSDK::ACK::ErrorCode pauseAck = mVehicle->missionManager->hpMission->pause(mFunctionTimeout);
        // mSecureGuard.unlock();
        // if (DJI::OSDK::ACK::getError(pauseAck)){
        //     DJI::OSDK::ACK::getErrorCodeMessage(pauseAck, __func__);
        //     LogStatus::get()->error("Error at pause mission, exiting", true);
        //     unsubscribeToData();     // 666 TODO: Make this better  
        //     return false;
        // }
        // sleep(5);

        // // Resume
        // LogStatus::get()->status("Resume mission", true);
        // mSecureGuard.lock();
        // DJI::OSDK::ACK::ErrorCode resumeAck = mVehicle->missionManager->hpMission->resume(mFunctionTimeout);
        // mSecureGuard.unlock();
        // if (DJI::OSDK::ACK::getError(resumeAck)){
        //     DJI::OSDK::ACK::getErrorCodeMessage(resumeAck, __func__);
        //     LogStatus::get()->error("Error at resume mission, exiting", true);
        //     unsubscribeToData();     // 666 TODO: Make this better  
        //     return false;
        // }
        // sleep(5);

        // // Update radius, no ACK
        // LogStatus::get()->status("Update radius to 1.5x: new radius = " + std::to_string(1.5 * _initialRadius), true);
        // mSecureGuard.lock();
        // mVehicle->missionManager->hpMission->updateRadius(1.5 * _initialRadius);
        // mSecureGuard.unlock();
        // sleep(5);

        // // Update velocity (yawRate), no ACK
        // LogStatus::get()->status("Update hotpoint rotation rate: new rate = 5 deg/s", true);
        // DJI::OSDK::HotpointMission::YawRate yawRateStruct;
        // yawRateStruct.clockwise = 1;
        // yawRateStruct.yawRate   = 5;
        // mSecureGuard.lock();
        // mVehicle->missionManager->hpMission->updateYawRate(yawRateStruct);
        // mSecureGuard.unlock();
        // sleep(5);

        // Stop
        LogStatus::get()->status("Stop mission", true);
        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode stopAck = mVehicle->missionManager->hpMission->stop(mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(stopAck)){
            DJI::OSDK::ACK::getErrorCodeMessage(stopAck, __func__);
            LogStatus::get()->error("Error at stop mission, exiting", true);
            unsubscribeToData();     // 666 TODO: Make this better  
            return false;
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
        mSecureGuard.lock();
        mVehicle = new DJI::OSDK::Vehicle(_config.device.c_str(),
                                   _config.baudrate,
                                   true,
                                   _config.useAdvancedSensing);

        // Check if the communication is working fine
        if (!mVehicle->protocolLayer->getDriver()->getDeviceStatus())
        {
            LogStatus::get()->error("Comms appear to be incorrectly set up exiting", true);
            return false;
        }
        mSecureGuard.unlock();

        // Activate
        mActivateData.ID = _config.app_id;
        char app_key[65];
        mActivateData.encKey = app_key;
        std::strcpy(mActivateData.encKey, _config.app_key.c_str());

        mSecureGuard.lock();
        mActivateData.version = mVehicle->getFwVersion();
        DJI::OSDK::ACK::ErrorCode ack = mVehicle->activate(&mActivateData, mFunctionTimeout);
        mSecureGuard.unlock();

        if (DJI::OSDK::ACK::getError(ack))
        {
            DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
            LogStatus::get()->error("Error validating Vehicle, exiting", true);
            return false;
        }

        if(mVehicle == NULL){
            LogStatus::get()->error("Vehicle not initialized, exiting", true);
            return false;
        }
        LogStatus::get()->status("Vehicle initialized", true);
      
        // Obtain Control Authority
        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode ctrlStatus = mVehicle->obtainCtrlAuthority(mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(ctrlStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(ctrlStatus, __func__);
            LogStatus::get()->error("Not obtained Control Authority, exiting", true);
            return false;
        }
        LogStatus::get()->status("Obtained Control Authority of the Vehicle", true);

        if(subscribeToData()){     // 666 TODO: MAKE THIS BETTER, UNA ESPECIE DE SINGLETON?
            LogStatus::get()->status("subscribe To Data success", true);
            return true;
        }else{
            LogStatus::get()->status("subscribe To Data failed", true);
            return false;
        }    
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
        
        mSecureGuard.lock();
        subscribeStatus = mVehicle->subscribe->verify(mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            LogStatus::get()->error("Not verified subscription to Telemetry, exiting", true);
            return false;
        }
	
        // Package 0: Subscribe to flight status and mode at freq 10 Hz
        mPkgIndex = 0;
        int freq = 10;
        DJI::OSDK::Telemetry::TopicName topicList10Hz[] = { DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT, DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE};
        int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        mSecureGuard.lock();
        bool pkgStatus = mVehicle->subscribe->initPackageFromTopicList(mPkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        mSecureGuard.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 0 from topic list, exiting", true);
            return false;
        }

        mSecureGuard.lock();
        subscribeStatus = mVehicle->subscribe->startPackage(mPkgIndex, mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            LogStatus::get()->error("Start package 0 error, exiting", true);
            return false;
        }

        // Package 1: Subscribe to Lat/Lon, Alt, RC Channel and Velocity at freq 50 Hz
        mPkgIndex = 1;
        freq = 50;
        DJI::OSDK::Telemetry::TopicName topicList50Hz[] = { DJI::OSDK::Telemetry::TOPIC_GPS_FUSED, DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED, DJI::OSDK::Telemetry::TOPIC_RC, DJI::OSDK::Telemetry::TOPIC_VELOCITY};
        numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
        enableTimestamp = false;

        mSecureGuard.lock();
        pkgStatus = mVehicle->subscribe->initPackageFromTopicList(mPkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
        mSecureGuard.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 1 from topic list, exiting", true);
            return false;
        }

        mSecureGuard.lock();
        subscribeStatus = mVehicle->subscribe->startPackage(mPkgIndex, mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            LogStatus::get()->error("Start package 1 error, exiting", true);
            return false;
        }

        // Package 2: Subscribe to Quaternion at freq 200 Hz.
        mPkgIndex = 2;
        freq = 200;
        DJI::OSDK::Telemetry::TopicName topicList200Hz[] = { DJI::OSDK::Telemetry::TOPIC_QUATERNION };
        numTopic = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
        enableTimestamp = false;

        mSecureGuard.lock();
        pkgStatus = mVehicle->subscribe->initPackageFromTopicList(mPkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
        mSecureGuard.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 2 from topic list, exiting", true);
            return false;
        }

        mSecureGuard.lock();
        subscribeStatus = mVehicle->subscribe->startPackage(mPkgIndex, mFunctionTimeout);
        mSecureGuard.unlock();
        if (DJI::OSDK::ACK::getError(subscribeStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            unsubscribeToData();     // 666 TODO: Make this better  
            LogStatus::get()->error("Start package 2 error, exiting", true);
            return false;
        }

        // Package 3: Subscribe to RTK at freq 5 Hz.
        mPkgIndex = 3;
        freq = 5;
        DJI::OSDK::Telemetry::TopicName topicListRTK5Hz[] = {DJI::OSDK::Telemetry::TOPIC_RTK_POSITION, DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO, DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO, DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY, DJI::OSDK::Telemetry::TOPIC_RTK_YAW};
        numTopic = sizeof(topicListRTK5Hz) / sizeof(topicListRTK5Hz[0]);
        enableTimestamp = false;

        mSecureGuard.lock();
        pkgStatus = mVehicle->subscribe->initPackageFromTopicList(mPkgIndex, numTopic, topicListRTK5Hz, enableTimestamp, freq);
        mSecureGuard.unlock();
        if (!pkgStatus){
            LogStatus::get()->error("Not init package 3 from topic list, exiting", true);
            return false;
        }else{
            mSecureGuard.lock();
            subscribeStatus = mVehicle->subscribe->startPackage(mPkgIndex, mFunctionTimeout);
            mSecureGuard.unlock();
            if(subscribeStatus.data == DJI::OSDK::ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE){
                LogStatus::get()->status("RTK Not Available", true);
                mRTKAvailable = false;
            }else{
                mRTKAvailable = true;
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

        return setLocalPosition();

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::unsubscribeToData(){

        for(unsigned i = 0; i < mPkgIndex; i++ ){
            mSecureGuard.lock();
            DJI::OSDK::ACK::ErrorCode ack = mVehicle->subscribe->removePackage(i, mFunctionTimeout);
            mSecureGuard.unlock();
            if (DJI::OSDK::ACK::getError(ack)){
                LogStatus::get()->error("Error unsubscribing at package: " + std::to_string(i) + "; please restart the drone/FC to get back to a clean state, exiting", true);
                return false;
            }
        }
        
        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendDJI::setLocalPosition(){

        mSecureGuard.lock();
        mOriginGPS = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
        mSecureGuard.unlock();
        
        // Get the broadcast GP since we need the height
        mSecureGuard.lock();
        mBroadcastGP = mVehicle->broadcast->getGlobalPosition();
        mSecureGuard.unlock();

        return true;

    }
    
    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::localOffsetFromGpsOffset(DJI::OSDK::Telemetry::Vector3f& _deltaNed, DJI::OSDK::Telemetry::Vector3f& _deltaEnu, void* _target, void* _origin){
    
        DJI::OSDK::Telemetry::GPSFused* subscriptionTarget = (DJI::OSDK::Telemetry::GPSFused*)_target;
        DJI::OSDK::Telemetry::GPSFused*  subscriptionOrigin = (DJI::OSDK::Telemetry::GPSFused*)_origin;
        double deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        double deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        _deltaNed.x = deltaLat * C_EARTH;
        _deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        _deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;

        // ENU ? NEED TO CHECK
        _deltaEnu.x = DEG2RAD * deltaLon * C_EARTH * cos(DEG2RAD * subscriptionTarget->latitude);
        _deltaEnu.y = DEG2RAD * deltaLat * C_EARTH;
        _deltaEnu.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;

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
    std::vector<DJI::OSDK::WayPointSettings> BackendDJI::createWaypoints(int _numWaypoints, DJI::OSDK::float64_t _distanceIncrement, DJI::OSDK::float32_t _startAlt){
    
         // Create Start Waypoint
        DJI::OSDK::WayPointSettings start_wp;
        setWaypointDefaults(&start_wp);

        // Global position retrieved via subscription
        mSecureGuard.lock();
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type subscribeGPosition = mVehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();;
        mSecureGuard.unlock();
        
        start_wp.latitude  = subscribeGPosition.latitude;
        start_wp.longitude = subscribeGPosition.longitude;
        start_wp.altitude  = _startAlt;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n", subscribeGPosition.latitude, subscribeGPosition.longitude, _startAlt);
        
        std::vector<DJI::OSDK::WayPointSettings> wpVector = generateWaypointsPolygon(&start_wp, _distanceIncrement, _numWaypoints);
        return wpVector;

    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<DJI::OSDK::WayPointSettings> BackendDJI::generateWaypointsPolygon(DJI::OSDK::WayPointSettings* _startData, DJI::OSDK::float64_t _increment, int _numWp){

        // Let's create a vector to store our waypoints in.
        std::vector<DJI::OSDK::WayPointSettings> wp_list;

        // Some calculation for the polygon
        float64_t extAngle = 2 * M_PI / _numWp;

        // First waypoint
        _startData->index = 0;
        wp_list.push_back(*_startData);

        // Iterative algorithm
        for (int i = 1; i < _numWp; i++)
        {
            DJI::OSDK::WayPointSettings  wp;
            DJI::OSDK::WayPointSettings* prevWp = &wp_list[i - 1];
            setWaypointDefaults(&wp);
            wp.index     = i;
            wp.latitude  = (prevWp->latitude + (_increment * cos(i * extAngle)));
            wp.longitude = (prevWp->longitude + (_increment * sin(i * extAngle)));
            wp.altitude  = (prevWp->altitude + 1);
            wp_list.push_back(wp);
        }

        // Come back home
        _startData->index = _numWp;
        wp_list.push_back(*_startData);

        return wp_list;

    }

    //-----------------------------------------------------------------------------------------------------------------
    void BackendDJI::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& _wpList){

        for (std::vector<DJI::OSDK::WayPointSettings>::iterator wp = _wpList.begin(); wp != _wpList.end(); ++wp){
            
            printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
            mSecureGuard.lock();
            DJI::OSDK::ACK::WayPointIndex wpDataACK = mVehicle->missionManager->wpMission->uploadIndexData(&(*wp), mFunctionTimeout);
            mSecureGuard.unlock();
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

        mSecureGuard.lock();
        DJI::OSDK::ACK::ErrorCode ack = mVehicle->broadcast->setBroadcastFreq(freq, 1);
        mSecureGuard.unlock();
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
