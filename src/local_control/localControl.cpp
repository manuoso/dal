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
    bool LocalControl::init(VectorPID _roll, VectorPID _pitch, VectorPID _yaw, VectorPID _z, VectorUtils _utils){

        // kp, ki, kd, minSat, maxSat, minWind, maxWind
        pidRoll_        = new PID(_roll[0], _roll[1], _roll[2], _roll[3], _roll[4], _roll[5], _roll[6]);

        pidPitch_       = new PID(_pitch[0], _pitch[1], _pitch[2], _pitch[3], _pitch[4], _pitch[5], _pitch[6]);

        pidYaw_         = new PID(_yaw[0], _yaw[1], _yaw[2], _yaw[3], _yaw[4], _yaw[5], _yaw[6]);

        pidZ_           = new PID(_z[0], _z[1], _z[2], _z[3], _z[4], _z[5], _z[6]);

        hoveringValue_  = _utils[0];

        maxRoll_        = _utils[1];
        maxPitch_       = _utils[2];
        maxWYaw_        = _utils[3];

        minThrotle_     = _utils[4];
        maxThrotle_     = _utils[5];

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR UPDATE
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    void LocalControl::reference(Eigen::Vector4f _xyzYaw){
        
        pidRoll_->reference(_xyzYaw[0]);
        pidPitch_->reference(_xyzYaw[1]);
        pidYaw_->reference(_xyzYaw[3]);
        pidZ_->reference(_xyzYaw[2]);

    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector4f LocalControl::update(Eigen::Vector4f _xyzYaw, float _incT){

        // Update PIDs
        float aY = pidRoll_->update(_xyzYaw[0], _incT);
        float aX = pidPitch_->update(_xyzYaw[1], _incT);
        float zPush = pidZ_->update(_xyzYaw[3], _incT);
        float wYaw = pidYaw_->update(_xyzYaw[2], _incT);

        // Transform target accelerations to roll pitch and thrust
        Eigen::Vector3f rpt = accelAngleConversion(aX, aY, zPush);
        
        // Return result
        Eigen::Vector4f result = {saturateSignal(rpt[0], maxRoll_),     // Output target roll.
                                saturateSignal(rpt[1], maxPitch_),      // Output target pitch.
                                saturateSignal(wYaw  , maxWYaw_),       // Output target w yaw.
                                saturateSignal(rpt[2], maxThrotle_)};   // Output target throtle.

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
        float roll = atan2(_aY,g);
        float pitch = -atan2(_aX*cos(roll), g);
        float thrust = _zPush + hoveringValue_;

        Eigen::Vector3f result = {roll, pitch, thrust}; 
        return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    float LocalControl::saturateSignal(float _signal, float _saturation){
        if(_signal < -_saturation){
            return -_saturation;
        }else if(_signal > _saturation){
            return _saturation;
        }else{
            return _signal;
        }
    }

}