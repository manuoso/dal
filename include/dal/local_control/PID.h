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


#ifndef DAL_LOCALCONTROL_PID_H_
#define DAL_LOCALCONTROL_PID_H_

#include <limits>
#include <thread>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace dal{
    class PID {
        public:
            /// Struct for Params of PID
            struct PIDParams{
                float kp = 0.0;
                float ki = 0.0; 
                float kd = 0.0;
                float sat = 0.0;
                float wind = 0.0;
            };

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
            //---------------------------------------------------------------------------------------------------------------------
            
            /// Constructor
            PID(float _kp, float _ki, float _kd,
                float _minSat = std::numeric_limits<float>::min(),
                float _maxSat = std::numeric_limits<float>::max(),
                float _minWind = std::numeric_limits<float>::min(),
                float _maxWind = std::numeric_limits<float>::max());

            /// Destructor
            ~PID();
            
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR UPDATE
            //---------------------------------------------------------------------------------------------------------------------

            /// This method update PID with actual pose.
            /// \param _val: actual value to update.
            /// \param _incT: difference between two iterations.
            /// \return error from PID.
            float update(float _val, float _incT);

            //---------------------------------------------------------------------------------------------------------------------
            // GETTERS
            //---------------------------------------------------------------------------------------------------------------------

            /// This method get the actual reference.
            /// \return actual reference.
            float reference() { return reference_; }

            /// This method get the actual kp.
            /// \return actual kp.
            float kp() const { return kp_; }

            /// This method get the actual ki.
            /// \return actual ki.
            float ki() const { return ki_; }

            /// This method get the actual kd.
            /// \return actual kd.
            float kd() const { return kd_; }

            /// This method get the actual saturations.
            /// \param _min: get actual min value.
            /// \param _max: get actual max value.
            /// \return void.
            void getSaturations(float& _min, float& _max) { _min = minSat_; _max = maxSat_; }
        
            /// This method get the actual wind-up.
            /// \param _min: get actual min value.
            /// \param _max: get actual max value.
            /// \return void.
            void getWindupTerms(float& _min, float& _max) { _min = windupMin_; _max = windupMax_; }
            
            //---------------------------------------------------------------------------------------------------------------------
            // SETTERS
            //---------------------------------------------------------------------------------------------------------------------

            /// This method set a new reference.
            /// \param _ref: new reference to set.
            /// \return void.
            void reference(float _ref) { reference_ = _ref; accumErr_ = 0; lastError_ = 0; lastResult_ = 0; bouncingFactor_ = 0.1; }
            
            /// This method set a new kp.
            /// \param _ref: new kp to set.
            /// \return void.
            void kp(float _kp) { kp_ = _kp; }

            /// This method set a new ki.
            /// \param _ref: new ki to set.
            /// \return void.
            void ki(float _ki) { ki_ = _ki; }

            /// This method set a new kd.
            /// \param _ref: new kd to set.
            /// \return void.
            void kd(float _kd) { kd_ = _kd; }

            /// This method set a new saturations value.
            /// \param _min: set min value.
            /// \param _max: set max value.
            /// \return void.
            void setSaturations(float _min, float _max) { minSat_ = _min; maxSat_ = _max; }

            /// This method set a new wind-up value.
            /// \param _min: set min value.
            /// \param _max: set max value.
            /// \return void.
            void setWindupTerms(float _min, float _max) { windupMin_ = _min; windupMax_ = _max; }

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
