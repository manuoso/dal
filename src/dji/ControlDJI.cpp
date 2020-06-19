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

#include <dal/dji/ControlDJI.h>

namespace dal{
    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    ControlDJI::ControlDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    ControlDJI::~ControlDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR CONTROL
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::recoverFromManual(){

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
    bool ControlDJI::emergencyBrake(){

        HAL::vehicle_->control->emergencyBrake();
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::arm(){
        std::cout << "\033[32mArming motors \033[m" << std::endl;

        DJI::OSDK::ACK::ErrorCode armStatus = HAL::vehicle_->control->armMotors(HAL::functionTimeout_);
        if(DJI::OSDK::ACK::getError(armStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(armStatus, __func__);
            std::cout << "\033[31mError at arm motors, exiting \033[m" << std::endl;
            return false;
        }
        
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::disarm(){
        std::cout << "\033[32mDisarming motors \033[m" << std::endl;

        DJI::OSDK::ACK::ErrorCode disarmStatus = HAL::vehicle_->control->disArmMotors(HAL::functionTimeout_);
        if(DJI::OSDK::ACK::getError(disarmStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(disarmStatus, __func__);
            std::cout << "\033[31mError at disarm motors, exiting \033[m" << std::endl;
            return false;
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::takeOff(const float _height){
        
        std::cout << "\033[32mStart takeoff \033[m" << std::endl;
        // Start takeoff
        DJI::OSDK::ACK::ErrorCode takeoffStatus = HAL::vehicle_->control->takeoff(HAL::functionTimeout_);
        if(DJI::OSDK::ACK::getError(takeoffStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(takeoffStatus, __func__);
            std::cout << "\033[31mError at start takeoff, exiting \033[m" << std::endl;
            return false;
        }

        // First check: Motors started
        int motorsNotStarted = 0;
        int timeoutCycles    = 20;

        while(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() != DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND &&
            HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START &&
            motorsNotStarted < timeoutCycles){
                
                motorsNotStarted++;
                usleep(100000);
        }

        if(motorsNotStarted == timeoutCycles){
            std::cout << "\033[31mTakeoff failed. Motors are not spinning, exiting \033[m" << std::endl;
            return false; 
        }else{
            std::cout << "\033[32mMotors spinning... \033[m" << std::endl;
        }
      
        // Second check: In air
        int stillOnGround = 0;
        timeoutCycles     = 110;

        while(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR &&
            (HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
            stillOnGround < timeoutCycles){
            
            stillOnGround++;
            usleep(100000);
        }

        if(stillOnGround == timeoutCycles){
            std::cout << "\033[31mTakeoff failed. Aircraft is still on the ground, but the motors are spinning, exiting \033[m" << std::endl;
            return false;
        }else{
            std::cout << "\033[32mAscending... \033[m" << std::endl;
        }

        // Final check: Finished takeoff
        while(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF){
                
                sleep(1);
        }

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
                
                std::cout << "\033[32mSuccessful takeoff! \033[m" << std::endl;
        }else{
            std::cout << "\033[31mTakeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting \033[m" << std::endl;
            return false;
        }

        
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::land(){
        
        std::cout << "\033[32mStart land \033[m" << std::endl;
        // Start landing
        DJI::OSDK::ACK::ErrorCode landingStatus = HAL::vehicle_->control->land(HAL::functionTimeout_);
        if(DJI::OSDK::ACK::getError(landingStatus) != DJI::OSDK::ACK::SUCCESS){
            DJI::OSDK::ACK::getErrorCodeMessage(landingStatus, __func__);
            std::cout << "\033[31mError at land, exiting \033[m" << std::endl;
            return false;
        }

        // First check: Landing started
        int landingNotStarted = 0;
        int timeoutCycles     = 20;

        while (HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            landingNotStarted < timeoutCycles){
                
                landingNotStarted++;
                usleep(100000);
        }

        if(landingNotStarted == timeoutCycles){
            std::cout << "\033[31mLanding failed. Aircraft is still in the air, exiting \033[m" << std::endl;
            return false;
        }else{
            std::cout << "\033[32mLanding... \033[m" << std::endl;
        }

        // Second check: Finished landing
        while (HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() == DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>() == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR){
                
                sleep(1);
        }

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS ||
            HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE){
                
                std::cout << "\033[32mSuccessful landing! \033[m" << std::endl;
        }else{
            std::cout << "\033[31mLanding finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting \033[m" << std::endl;
            return false;
        }
        
        return true;        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::position(float _x, float _y, float _z, float _yaw){

        if(controlAct_){
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }
        
        // Get local position
        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();

        Eigen::Vector3f localPoseGPS;
        localPoseFromGps(localPoseGPS, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&HAL::originGPS_));
        
        // 666 TODO: CHANGE GPS FUSED ALTITUDE

        // Get initial offset. We will update this in a loop later.
        double xOffset = _x - localPoseGPS[0];
        double yOffset = _y - localPoseGPS[1];
        double zOffset = _z - localPoseGPS[2];

        // 0.1 m or 10 cms is the minimum error to reach target in x, y and z axes.
        // This error threshold will have to change depending on aircraft / payload / wind conditions
        double xCmd, yCmd, zCmd;
        if(((std::abs(xOffset)) < 0.1) && ((std::abs(yOffset)) < 0.1) && (localPoseGPS(2) > (zOffset - 0.1)) && (localPoseGPS(2) < (zOffset + 0.1))){
            xCmd = 0;
            yCmd = 0;
            zCmd = 0;
        }else{
            xCmd = xOffset;
            yCmd = yOffset;
            zCmd = _z;
        }

        // 666 TODO: YAW NOT IMPLEMENTED!

        HAL::vehicle_->control->positionAndYawCtrl(xCmd, yCmd, zCmd, _yaw);

        controlAct_ = false;

        return true;
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::velocity(float _vx, float _vy, float _vz, float _yawRate){
        
        if(controlAct_){
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }

        HAL::vehicle_->control->velocityAndYawRateCtrl(_vx, _vy, _vz, _yawRate);
        
        controlAct_ = false;

        return true;
    }    


    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::attitude(float _roll, float _pitch, float _yaw, float _z){

        if(controlAct_){
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }

        HAL::vehicle_->control->attitudeAndVertPosCtrl(_roll, _pitch, _yaw, _z);
        
        controlAct_ = false;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::attitudeRate(float _rollRate, float _pitchRate, float _yawRate, float _z){

        if(controlAct_){
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }

        HAL::vehicle_->control->angularRateAndVertPosCtrl(_rollRate, _pitchRate, _yawRate, _z);
        
        controlAct_ = false;
        
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::rpyThrust(float _roll, float _pitch, float _yawRate, float _thrust){
        
        // uint8_t mode =  (0 << 0)+               // bit 0 non stable mode
        //                 (1 << 1) + (0 << 2)+    // bit 2:1 body frame
        //                 (1 << 3)+               // bit 3 yaw rate
        //                 (0 << 4) + (1 << 5)+    // bit 5:4 vertical thrust
        //                 (0 << 6);               // bit 7:6 horizontal atti
        uint flag = DJI::OSDK::Control::HorizontalLogic::HORIZONTAL_ANGLE |
                    DJI::OSDK::Control::VerticalLogic::VERTICAL_THRUST |
                    DJI::OSDK::Control::YawLogic::YAW_RATE |
                    DJI::OSDK::Control::HorizontalCoordinate::HORIZONTAL_BODY |
                    DJI::OSDK::Control::StableMode::STABLE_DISABLE;

        return customControl(flag, _roll, _pitch, _thrust, _yawRate);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool ControlDJI::customControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP){

        if(controlAct_){
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE &&
        HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>() != DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL){
            
            int mode = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }
        
        // FROM DJI ROS ONBOARD SDK
        uint8_t HORI  = (_flag & 0xC0);
        uint8_t VERT  = (_flag & 0x30);
        uint8_t YAW   = (_flag & 0x08);
        uint8_t FRAME = (_flag & 0x06);
        // uint8_t HOLD  = (_flag & 0x01);

        double xCmd, yCmd, zCmd, yawCmd;
        if (FRAME == DJI::OSDK::Control::HORIZONTAL_GROUND)
        {
            // 1.1 Horizontal channels
            if ( (HORI == DJI::OSDK::Control::HORIZONTAL_VELOCITY) || (HORI == DJI::OSDK::Control::HORIZONTAL_POSITION) )
            {
            xCmd = _ySP;
            yCmd = _xSP;
            }
            else
            {
            xCmd = RAD2DEG(_xSP);
            yCmd = RAD2DEG(-_ySP);
            }

            // 1.2 Verticle Channel
            if ( (VERT == DJI::OSDK::Control::VERTICAL_VELOCITY) || (VERT == DJI::OSDK::Control::VERTICAL_POSITION) )
            {
            zCmd = _zSP;
            }
            else
            {
            zCmd = _zSP;
            }
        }
        else if(FRAME == DJI::OSDK::Control::HORIZONTAL_BODY)
        {
            // 2.1 Horizontal channels
            if ( (HORI == DJI::OSDK::Control::HORIZONTAL_VELOCITY) || (HORI == DJI::OSDK::Control::HORIZONTAL_POSITION) )
            {
            // The X and Y Vel and Pos should be only based on rotation after Yaw,
            // whithout roll and pitch. Otherwise the behavior will be weird.

            // Transform from F-R to F-L
            xCmd = _xSP;
            yCmd = -_ySP;
            }
            else
            {
            xCmd = RAD2DEG(_xSP);
            yCmd = RAD2DEG(-_ySP);
            }

            // 2.2 Vertical channel
            if ( (VERT == DJI::OSDK::Control::VERTICAL_VELOCITY) || (VERT == DJI::OSDK::Control::VERTICAL_POSITION)  )
            {
            zCmd = _zSP;
            }
            else
            {
            zCmd = _zSP;
            }
        }

        // The behavior of yaw should be the same in either frame
        if ( YAW == DJI::OSDK::Control::YAW_ANGLE )
        {
            Eigen::Matrix3d R_FLU2FRD;
            R_FLU2FRD << 1,  0,  0, 0, -1,  0, 0,  0, -1;
            Eigen::Matrix3d R_ENU2NED;
            R_ENU2NED << 0,  1,  0, 1,  0,  0, 0,  0, -1;  
            
            Eigen::AngleAxisd rollAngle(0.0  , Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(0.0 , Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(_yawSP, Eigen::Vector3d::UnitZ());

            Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
            Eigen::Matrix3d rotationSrc = q.matrix();

            //The last term should be transpose, but since it's symmetric ...
            Eigen::Matrix3d rotationDes;
            rotationDes = R_ENU2NED * rotationSrc * R_FLU2FRD;

            Eigen::Vector3d ea = rotationDes.eulerAngles(0,1,2);
            yawCmd = ea[2];

            yawCmd = RAD2DEG(yawCmd);
        }
        else if (YAW == DJI::OSDK::Control::YAW_RATE)
        {
            yawCmd = RAD2DEG(-_yawSP);
        }

        DJI::OSDK::Control::CtrlData ctrlData(_flag, xCmd, yCmd, zCmd, yawCmd);
        HAL::vehicle_->control->flightCtrl(ctrlData);

        controlAct_ = false;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    void ControlDJI::localPoseFromGps(Eigen::Vector3f& _delta, void* _target, void* _origin){
    
        DJI::OSDK::Telemetry::GPSFused* subscriptionTarget = (DJI::OSDK::Telemetry::GPSFused*)_target;
        DJI::OSDK::Telemetry::GPSFused*  subscriptionOrigin = (DJI::OSDK::Telemetry::GPSFused*)_origin;
        
        double t_lon = subscriptionTarget->longitude * 180.0 / C_PI;
        double r_lon = subscriptionOrigin->longitude * 180.0 / C_PI;
        
        double t_lat = subscriptionTarget->latitude * 180.0 / C_PI;
        double r_lat = subscriptionOrigin->latitude * 180.0 / C_PI;

        double deltaLon   = t_lon - r_lon;
        double deltaLat   = t_lat - r_lat;

        // NEU -> North East Up
        _delta[0] = DEG2RAD(deltaLon) * C_EARTH * cos(DEG2RAD(t_lat));
        _delta[1] = DEG2RAD(deltaLat) * C_EARTH;
        _delta[2] = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }

}
