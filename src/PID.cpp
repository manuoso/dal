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


#include <dal/PID.h>

namespace dal{
    //---------------------------------------------------------------------------------------------------------------------
    PID::PID(float _kp, float _ki, float _kd, float _minSat, float _maxSat, float _minWind, float _maxWind) {
        kp_ = _kp;
        ki_ = _ki;
        kd_ = _kd;
        minSat_ = _minSat;
        maxSat_ = _maxSat;
        windupMin_ = _minWind;
        windupMax_ = _maxWind;
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    float PID::update(float & _val, float _incT) {
        float dt = _incT; // TODO 666 input arg?

        // Calculate error
        float err = reference_ - _val;

        accumErr_ += err*dt;
        // Apply anti wind-up 777 Analyze other options
        accumErr_ = std::min(std::max(accumErr_, windupMin_), windupMax_);

        // Compute PID
        lastResult_ = kp_*err + ki_*accumErr_ + kd_*(err- lastError_)/dt;
        lastError_ = err;

        // Saturate signal
        lastResult_ = std::min(std::max(lastResult_, minSat_), maxSat_);
        lastResult_ *= bouncingFactor_;

        bouncingFactor_ *= 2.0;
        bouncingFactor_ = bouncingFactor_ > 1.0 ? 1.0 : bouncingFactor_;

        return lastResult_;
    }
}