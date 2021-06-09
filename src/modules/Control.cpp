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

#include <dal/modules/Control.hpp>

namespace dal     {
namespace modules {

    using namespace DJI::OSDK;
    using namespace DJI::OSDK::Telemetry;

    // ----------------------------------------------------------------------
    Control::Control(std::shared_ptr<HAL> & _hal) 
        : functionTimeout_(1)
        , controlAct_(false)
    {
        hal_ = _hal;
        if (hal_ != nullptr)
            started_ = true;
        else
            started_ = false;
    }

    // ----------------------------------------------------------------------
    Control::~Control()
    {
        this->stop();
    }

    // ----------------------------------------------------------------------
    void Control::stop()
    {
        if (started_)
        {
            started_ = false;
            condVar_.notify_all();
        }
    }

    // ----------------------------------------------------------------------
    bool Control::recoverFromManual()
    {
        if (!started_)
            return false;

        ACK::ErrorCode ctrlStatus = hal_->getVehicle()->obtainCtrlAuthority(functionTimeout_);
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
    bool Control::releaseAuthority()
    {
        if (!started_)
            return false;

        ACK::ErrorCode ctrlStatus = hal_->getVehicle()->releaseCtrlAuthority(functionTimeout_);
        if (ACK::getError(ctrlStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(ctrlStatus, __func__);
            std::cout << "\033[31m Cannot release DJI authority \033[m" << std::endl;
            return false;
        }
        std::cout << "\033[32m DJI Authority released. Going back to manual flight \033[m" << std::endl;
        return true;
    }

    // ----------------------------------------------------------------------
    void Control::emergencyBrake()
    {
        if (!started_)
            return;

        hal_->getVehicle()->control->emergencyBrake();
    }

    // ----------------------------------------------------------------------
    bool Control::arm()
    {
        if (!started_)
            return false;

        std::cout << "\033[32mArming motors \033[m" << std::endl;

        ACK::ErrorCode armStatus = hal_->getVehicle()->control->armMotors(functionTimeout_);
        if(ACK::getError(armStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(armStatus, __func__);
            std::cout << "\033[31mError at arm motors, exiting \033[m" << std::endl;
            return false;
        }
        return true;
    }
    
    // ----------------------------------------------------------------------
    bool Control::disarm()
    {
        if (!started_)
            return false;

        std::cout << "\033[32mDisarming motors \033[m" << std::endl;

        ACK::ErrorCode disarmStatus = hal_->getVehicle()->control->disArmMotors(functionTimeout_);
        if(ACK::getError(disarmStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(disarmStatus, __func__);
            std::cout << "\033[31mError at disarm motors, exiting \033[m" << std::endl;
            return false;
        }
        return true;
    }

    // ----------------------------------------------------------------------
    bool Control::takeOff(bool _block)
    {
        if (!started_)
            return false;

        if (_block)
        {
            launchTakeoff(nullptr);
        }
        else
        {
            SET_ACTION_CALLBACK(cb, condVar_)
            auto nonBlock = std::async(std::launch::async, &Control::launchTakeoff, this, std::move(cb));
            WAIT_ACTION(condVar_, started_)
        }

        return true;
    }

    // ----------------------------------------------------------------------
    bool Control::land(bool _block)
    {
        if (!started_)
            return false;
        
        if (_block)
        {
            launchLand(nullptr);
        }
        else
        {
            SET_ACTION_CALLBACK(cb, condVar_)
            auto nonBlock = std::async(std::launch::async, &Control::launchLand, this, std::move(cb));
            WAIT_ACTION(condVar_, started_)
        }

        return true;
    }

    // ----------------------------------------------------------------------
    bool Control::position(float _x, float _y, float _z, float _yaw)
    {
        if (!started_)
            return false;

        if(controlAct_)
        {
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_P_GPS &&
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ATTITUDE &&
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL)
        {
            int mode = hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }
        
        // Get local position
        TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS = hal_->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();
        auto originGPS = hal_->getOriginGPS();

        std::vector<float> localPoseGPS;
        localPoseFromGps(localPoseGPS, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&originGPS));
        
        // 666 TODO: CHANGE GPS FUSED ALTITUDE

        // Get initial offset. We will update this in a loop later.
        double xOffset = _x - localPoseGPS[0];
        double yOffset = _y - localPoseGPS[1];
        double zOffset = _z - localPoseGPS[2];

        // 0.1 m or 10 cms is the minimum error to reach target in x, y and z axes.
        // This error threshold will have to change depending on aircraft / payload / wind conditions
        double xCmd, yCmd, zCmd;
        if(((std::abs(xOffset)) < 0.1) && 
            ((std::abs(yOffset)) < 0.1) && 
            (localPoseGPS[2] > (zOffset - 0.1)) && 
            (localPoseGPS[2] < (zOffset + 0.1)))
        {
            xCmd = 0;
            yCmd = 0;
            zCmd = 0;
        }else
        {
            xCmd = xOffset;
            yCmd = yOffset;
            zCmd = _z;
        }

        // 666 TODO: YAW NOT IMPLEMENTED!

        hal_->getVehicle()->control->positionAndYawCtrl(xCmd, yCmd, zCmd, _yaw);

        controlAct_ = false;

        return true;
    }

    // ----------------------------------------------------------------------
    bool Control::velocity(float _vx, float _vy, float _vz, float _yawRate)
    {
        if (!started_)
            return false;

        uint flag = ::Control::HorizontalLogic::HORIZONTAL_VELOCITY |
                    ::Control::VerticalLogic::VERTICAL_VELOCITY |
                    ::Control::YawLogic::YAW_RATE |
                    ::Control::HorizontalCoordinate::HORIZONTAL_BODY |
                    ::Control::StableMode::STABLE_ENABLE;

        return customControl(flag, _vx, _vy, _vz, _yawRate);
    }

    // ----------------------------------------------------------------------
    bool Control::rpyThrust(float _roll, float _pitch, float _yawRate, float _thrust)
    {
        if (!started_)
            return false;

        // uint8_t mode =  (0 << 0)+               // bit 0 non stable mode
        //                 (1 << 1) + (0 << 2)+    // bit 2:1 body frame
        //                 (1 << 3)+               // bit 3 yaw rate
        //                 (0 << 4) + (1 << 5)+    // bit 5:4 vertical thrust
        //                 (0 << 6);               // bit 7:6 horizontal atti
        uint flag = ::Control::HorizontalLogic::HORIZONTAL_ANGLE |
                    ::Control::VerticalLogic::VERTICAL_THRUST |
                    ::Control::YawLogic::YAW_RATE |
                    ::Control::HorizontalCoordinate::HORIZONTAL_BODY |
                    ::Control::StableMode::STABLE_DISABLE;

        return customControl(flag, _roll, _pitch, _thrust, _yawRate);
    }

    // ----------------------------------------------------------------------
    bool Control::customControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP)
    {
        if (!started_)
            return false;

        if(controlAct_)
        {
            std::cout << "\033[31mError, you are trying to use several control functions at the same time\033[m" << std::endl;
            return false;
        }
        controlAct_ = true;

        if(hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_P_GPS &&
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ATTITUDE &&
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL)
        {
            int mode = hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();
            std::cout << "\033[31mError vehicle not in mode to control with API SDK, mode: \033[m" + std::to_string(mode) << std::endl;
            return false;
        }
        
        // FROM DJI ROS ONBOARD SDK
        uint8_t HORI  = (_flag & 0xC0);
        uint8_t VERT  = (_flag & 0x30);
        uint8_t YAW   = (_flag & 0x08);
        uint8_t FRAME = (_flag & 0x06);
        // uint8_t HOLD  = (_flag & 0x01);

        double xCmd = 0, yCmd = 0, zCmd = 0, yawCmd = 0;
        if (FRAME == ::Control::HORIZONTAL_GROUND)
        {
            // 1.1 Horizontal channels
            if ( (HORI == ::Control::HORIZONTAL_VELOCITY) || 
                (HORI == ::Control::HORIZONTAL_POSITION) )
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
            if ( (VERT == ::Control::VERTICAL_VELOCITY) || 
                (VERT == ::Control::VERTICAL_POSITION) )
            {
                zCmd = _zSP;
            }
            else
            {
                zCmd = _zSP;
            }
        }
        else if(FRAME == ::Control::HORIZONTAL_BODY)
        {
            // 2.1 Horizontal channels
            if ( (HORI == ::Control::HORIZONTAL_VELOCITY) || 
                (HORI == ::Control::HORIZONTAL_POSITION) )
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
            if ( (VERT == ::Control::VERTICAL_VELOCITY) || 
                (VERT == ::Control::VERTICAL_POSITION)  )
            {
                zCmd = _zSP;
            }
            else
            {
                zCmd = _zSP;
            }
        }

        // The behavior of yaw should be the same in either frame
        if (YAW == ::Control::YAW_ANGLE )
        {
            // TODO:
            // TODO:
            // TODO: YAW CONVERSION WITHOUT EIGEN
            // TODO:
            // TODO:

            // Eigen::Matrix3d R_FLU2FRD;
            // R_FLU2FRD << 1,  0,  0, 0, -1,  0, 0,  0, -1;
            // Eigen::Matrix3d R_ENU2NED;
            // R_ENU2NED << 0,  1,  0, 1,  0,  0, 0,  0, -1;  
            
            // Eigen::AngleAxisd rollAngle(0.0  , Eigen::Vector3d::UnitX());
            // Eigen::AngleAxisd pitchAngle(0.0 , Eigen::Vector3d::UnitY());
            // Eigen::AngleAxisd yawAngle(_yawSP, Eigen::Vector3d::UnitZ());

            // Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
            // Eigen::Matrix3d rotationSrc = q.matrix();

            // //The last term should be transpose, but since it's symmetric ...
            // Eigen::Matrix3d rotationDes;
            // rotationDes = R_ENU2NED * rotationSrc * R_FLU2FRD;

            // Eigen::Vector3d ea = rotationDes.eulerAngles(0,1,2);
            // yawCmd = ea[2];

            yawCmd = RAD2DEG(yawCmd);
        }
        else if (YAW == ::Control::YAW_RATE)
        {
            yawCmd = RAD2DEG(-_yawSP);
        }

        ::Control::CtrlData ctrlData(_flag, xCmd, yCmd, zCmd, yawCmd);
        hal_->getVehicle()->control->flightCtrl(ctrlData);

        controlAct_ = false;

        return true;
    }

    // ----------------------------------------------------------------------
    void Control::launchTakeoff(std::function<void(int)> _cb)
    {
        std::cout << "\033[32mStart takeoff \033[m" << std::endl;
        ACK::ErrorCode takeoffStatus = hal_->getVehicle()->control->takeoff(functionTimeout_);
        if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(takeoffStatus, __func__);
            std::cout << "\033[31mError at start takeoff, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return;
        }

        // First check: Motors started
        int motorsNotStarted = 0;
        int timeoutCycles    = 20;

        while (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() != VehicleStatus::FlightStatus::ON_GROUND &&
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ENGINE_START &&
            motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(100000);
        }

        if (motorsNotStarted == timeoutCycles)
        {
            std::cout << "\033[31mTakeoff failed. Motors are not spinning, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return; 
        }
        else
        {
            std::cout << "\033[32mMotors spinning... \033[m" << std::endl;
        }
      
        // Second check: In air
        int stillOnGround = 0;
        timeoutCycles     = 110;

        while (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() != VehicleStatus::FlightStatus::IN_AIR &&
            (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
            stillOnGround < timeoutCycles)
        {    
            stillOnGround++;
            usleep(100000);
        }

        if (stillOnGround == timeoutCycles)
        {
            std::cout << "\033[31mTakeoff failed. Aircraft is still on the ground, but the motors are spinning, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return;
        }
        else
        {
            std::cout << "\033[32mAscending... \033[m" << std::endl;
        }

        // Final check: Finished takeoff
        while (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
        {        
                sleep(1);
        }

        if (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_P_GPS ||
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {        
                std::cout << "\033[32mSuccessful takeoff! \033[m" << std::endl;
        }
        else
        {
            std::cout << "\033[31mTakeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return;
        }
        if (_cb)
            _cb(1);
    }

    // ----------------------------------------------------------------------
    void Control::launchLand(std::function<void(int)> _cb)
    {
        std::cout << "\033[32mStart land \033[m" << std::endl;
        ACK::ErrorCode landingStatus = hal_->getVehicle()->control->land(functionTimeout_);
        if (ACK::getError(landingStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(landingStatus, __func__);
            std::cout << "\033[31mError at land, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return;
        }

        // First check: Landing started
        int landingNotStarted = 0;
        int timeoutCycles     = 20;

        while (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            landingNotStarted < timeoutCycles)
        {        
                landingNotStarted++;
                usleep(100000);
        }

        if (landingNotStarted == timeoutCycles)
        {
            std::cout << "\033[31mLanding failed. Aircraft is still in the air, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return;
        }
        else
        {
            std::cout << "\033[32mLanding... \033[m" << std::endl;
        }

        // Second check: Finished landing
        while (hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() == VehicleStatus::FlightStatus::IN_AIR)
        {  
                sleep(1);
        }

        if(hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_P_GPS ||
            hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {        
                std::cout << "\033[32mSuccessful landing! \033[m" << std::endl;
        }
        else
        {
            std::cout << "\033[31mLanding finished, but the aircraft is in an unexpected mode. Please connect DJI GO, exiting \033[m" << std::endl;
            if (_cb)
                _cb(0);
            return;
        }
        if (_cb)
            _cb(1);
    }

}
}
