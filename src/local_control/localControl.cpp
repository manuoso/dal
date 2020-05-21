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


#include <dal/local_control/localControl.h>

namespace dal{

    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    LocalControl::LocalControl() {};

    //---------------------------------------------------------------------------------------------------------------------
    LocalControl::~LocalControl() {};

    //---------------------------------------------------------------------------------------------------------------------
    bool LocalControl::init(std::vector<float> _roll, std::vector<float> _pitch, std::vector<float> _yaw, std::vector<float> _z, std::string _typeAWU, std::vector<float> _utils){

        if(_utils.size() > 0 && _roll.size() > 0 && _pitch.size() > 0 && _yaw.size() > 0 && _z.size() > 0){
            std::cout << "\033[33mUsing new PIDs Values \033[m" << std::endl;

            // kp, ki, kd, minSat, maxSat
            pidRoll_        = new pidpp::PID(_roll[0], _roll[1], _roll[2], _roll[3], _roll[4]);
            pidPitch_       = new pidpp::PID(_pitch[0], _pitch[1], _pitch[2], _pitch[3], _pitch[4]);
            pidYaw_         = new pidpp::PID(_yaw[0], _yaw[1], _yaw[2], _yaw[3], _yaw[4]);
            pidZ_           = new pidpp::PID(_z[0], _z[1], _z[2], _z[3], _z[4]);

            if(_typeAWU == "none"){
                std::cout << "\033[33mUsing no AntiWindUp \033[m" << std::endl;
            }else if(_typeAWU == "sat"){
                std::cout << "\033[33mUsing Saturation AntiWindUp \033[m" << std::endl;
                pidRoll_->setAntiWindup(pidpp::PID::AntiWindupMethod::Saturation, {_roll[5], _roll[6]});
                pidPitch_->setAntiWindup(pidpp::PID::AntiWindupMethod::Saturation, {_pitch[5], _pitch[6]});
                pidYaw_->setAntiWindup(pidpp::PID::AntiWindupMethod::Saturation, {_yaw[5], _yaw[6]});
                pidZ_->setAntiWindup(pidpp::PID::AntiWindupMethod::Saturation, {_z[5], _z[6]});
            }else if(_typeAWU == "back_cal"){
                std::cout << "\033[33mUsing Back Calculation AntiWindUp \033[m" << std::endl;
                pidRoll_->setAntiWindup(pidpp::PID::AntiWindupMethod::BackCalculation, {_roll[5]});
                pidPitch_->setAntiWindup(pidpp::PID::AntiWindupMethod::BackCalculation, {_pitch[5]});
                pidYaw_->setAntiWindup(pidpp::PID::AntiWindupMethod::BackCalculation, {_yaw[5]}); 
                pidZ_->setAntiWindup(pidpp::PID::AntiWindupMethod::BackCalculation, {_z[5]});
            }else if(_typeAWU == "clamp"){
                std::cout << "\033[33mUsing Clamping AntiWindUp \033[m" << std::endl;
                pidRoll_->setAntiWindup(pidpp::PID::AntiWindupMethod::Clamping, {});
                pidPitch_->setAntiWindup(pidpp::PID::AntiWindupMethod::Clamping, {});
                pidYaw_->setAntiWindup(pidpp::PID::AntiWindupMethod::Clamping, {});
                pidZ_->setAntiWindup(pidpp::PID::AntiWindupMethod::Clamping, {});
            }else{
                std::cout << "\033[31mUnrecognized type of AntiWindUp \033[m" << std::endl;
                return false;
            }

            hoveringValue_  = _utils[0];
            massUAV_        = _utils[1];

            maxRoll_        = _utils[2];
            maxPitch_       = _utils[3];
            maxWYaw_        = _utils[4];

            minThrotle_     = _utils[5];
            maxThrotle_     = _utils[6];
        }else{
            std::cout << "\033[33mUsing default PIDs Values \033[m" << std::endl;

            pidRoll_        = new pidpp::PID(4, 1, 2, -100, 100);
            pidPitch_       = new pidpp::PID(4, 1, 2, -100, 100);
            pidYaw_         = new pidpp::PID(0.5, 0, 0.25, -3, 3);
            pidZ_           = new pidpp::PID(15, 1, 10, 0, 100);

            // 666 TODO: make a default PID values with AntiWindUp 

            hoveringValue_  = 18;
            massUAV_        = 1.9;

            // In rad
            maxRoll_        = 0.15;
            maxPitch_       = 0.15;
            maxWYaw_        = 1;

            minThrotle_     = 0;
            maxThrotle_     = 100;
        }
        
        std::cout << "\033[33mUtils values: \033[m" << "Hovering value: " << hoveringValue_ << " Mass UAV: " << massUAV_ << " Max Roll: " << maxRoll_ << " Max pitch: " << maxPitch_ << " Max Yaw: " << maxWYaw_ << " Min Throtle: " << minThrotle_ << " Max Throtle: " << maxThrotle_ << std::endl;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR CHANGE PIDS
    //---------------------------------------------------------------------------------------------------------------------
    bool LocalControl::changePID(std::string _type, PIDParams _params){
        if(_type == "roll"){
            if(_params.kp != -1){
                pidRoll_->kp(_params.kp);
                std::cout << "\033[33mNew Kp Value for Roll: \033[m" << _params.kp << std::endl;
            }
            if(_params.ki != -1){
                pidRoll_->ki(_params.ki);
                std::cout << "\033[33mNew Ki Value for Roll: \033[m" << _params.ki << std::endl;
            }
            if(_params.kd != -1){
                pidRoll_->kd(_params.kd);
                std::cout << "\033[33mNew Kd Value for Roll: \033[m" << _params.kd << std::endl;
            }
        }else if(_type == "pitch"){
            if(_params.kp != -1){
                pidPitch_->kp(_params.kp);
                std::cout << "\033[33mNew Kp Value for Pitch: \033[m" << _params.kp << std::endl;
            }
            if(_params.ki != -1){
                pidPitch_->ki(_params.ki);
                std::cout << "\033[33mNew Ki Value for Pitch: \033[m" << _params.ki << std::endl;
            }
            if(_params.kd != -1){
                pidPitch_->kd(_params.kd);
                std::cout << "\033[33mNew Kd Value for Pitch: \033[m" << _params.kd << std::endl;
            }
        }else if(_type == "yaw"){
            if(_params.kp != -1){
                pidYaw_->kp(_params.kp);
                std::cout << "\033[33mNew Kp Value for Yaw: \033[m" << _params.kp << std::endl;
            }
            if(_params.ki != -1){
                pidYaw_->ki(_params.ki);
                std::cout << "\033[33mNew Ki Value for Yaw: \033[m" << _params.ki << std::endl;
            }
            if(_params.kd != -1){
                pidYaw_->kd(_params.kd);
                std::cout << "\033[33mNew Kd Value for Yaw: \033[m" << _params.kd << std::endl;
            }
        }else if(_type == "z"){
            if(_params.kp != -1){
                pidZ_->kp(_params.kp);
                std::cout << "\033[33mNew Kp Value for Z: \033[m" << _params.kp << std::endl;
            }
            if(_params.ki != -1){
                pidZ_->ki(_params.ki);
                std::cout << "\033[33mNew Ki Value for Z: \033[m" << _params.ki << std::endl;
            }
            if(_params.kd != -1){
                pidZ_->kd(_params.kd);
                std::cout << "\033[33mNew Kd Value for Z: \033[m" << _params.kd << std::endl;
            }
        }else{
            std::cout << "\033[31mUnrecognized PID type \033[m" << std::endl;
            return false;
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR UPDATE
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool LocalControl::reference(Eigen::Vector4f _xyzYaw, bool _reset){
        
        pidPitch_->reference(_xyzYaw[0], _reset);
        pidRoll_->reference(_xyzYaw[1], _reset);
        pidZ_->reference(_xyzYaw[2], _reset);
        pidYaw_->reference(_xyzYaw[3], _reset);

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector4f LocalControl::update(Eigen::Vector4f _xyzYaw, float _incT){

        // Update PIDs
        float aX = pidPitch_->update(_xyzYaw[0], _incT);
        float aY = pidRoll_->update(_xyzYaw[1], _incT);
        float zPush = pidZ_->update(_xyzYaw[2], _incT);
        float wYaw = pidYaw_->update(_xyzYaw[3], _incT);

        // Transform target accelerations to roll pitch and thrust
        Eigen::Vector3f rpt = accelAngleConversion(aX, aY, zPush);
        
        // Return result
        Eigen::Vector4f result = {saturateSignal(rpt[0], -1*maxRoll_, maxRoll_),    // Output target roll.
                                saturateSignal(rpt[1], -1*maxPitch_, maxPitch_),    // Output target pitch.
                                saturateSignal(wYaw, -1*maxWYaw_, maxWYaw_),        // Output target w yaw.
                                saturateSignal(rpt[2], minThrotle_, maxThrotle_)};  // Output target throtle.

        return result;
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f LocalControl::accelAngleConversion(float _aX, float _aY, float _zPush){
        // m en Kg
        // a en m/s2
        const float g = 9.81;
        float roll = atan2(-_aY, g);
        float pitch = atan2(_aX*cos(roll), g);
        float thrust = _zPush + hoveringValue_;
        // float thrust = ((_zPush + g)*massUAV_)/(cos(rollLast_)*cos(pitchLast_)) + hoveringValue_;

        // rollLast_ = roll;
        // pitchLast_ = pitch;

        Eigen::Vector3f result = {roll, pitch, thrust}; 
        return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    float LocalControl::saturateSignal(float _signal, float _min, float _max){
        if(_signal < _min){
            return _min;
        }else if(_signal > _max){
            return _max;
        }else{
            return _signal;
        }
    }

}
