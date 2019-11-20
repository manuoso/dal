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

#ifndef DAL_PID_H_
#define DAL_PID_H_

#include <limits>
#include <algorithm>
#include <chrono>

namespace dal{
    class PID {
        public:
            //---------------------------------------------------------------------------------------------------------------------
            PID(float _kp, float _ki, float _kd,
                float _minSat = std::numeric_limits<float>::min(),
                float _maxSat = std::numeric_limits<float>::max(),
                float _minWind = std::numeric_limits<float>::min(),
                float _maxWind = std::numeric_limits<float>::max());

            //---------------------------------------------------------------------------------------------------------------------
            float update(float &_val, float _incT);

            //---------------------------------------------------------------------------------------------------------------------
            void reference(float _ref) { reference_ = _ref; accumErr_ = 0; lastError_ = 0; lastResult_ = 0; bouncingFactor_ = 0.1;}
            float reference() { return reference_; }
            
            //---------------------------------------------------------------------------------------------------------------------
            void kp(float _kp) { kp_ = _kp; }
            void ki(float _ki) { ki_ = _ki; }
            void kd(float _kd) { kd_ = _kd; }

            //---------------------------------------------------------------------------------------------------------------------
            float kp() const { return kp_; }    
            float ki() const { return ki_; }
            float kd() const { return kd_; }

            //---------------------------------------------------------------------------------------------------------------------
            void setSaturations(float _min, float _max) { minSat_ = _min; maxSat_ = _max; }
            void getSaturations(float _min, float _max) { _min = minSat_; _max = maxSat_; }

            //---------------------------------------------------------------------------------------------------------------------
            void setWindupTerms(float _min, float _max) { windupMin_ = _min; windupMax_ = _max; }
            void getWindupTerms(float _min, float _max) { _min = windupMin_; _max = windupMax_; }

        private:
            float reference_;
            float kp_, ki_, kd_;
            float minSat_, maxSat_;
            float windupMin_, windupMax_;

            float lastResult_, lastError_, accumErr_;

            double bouncingFactor_ = 0.1;

    };
}

#endif
