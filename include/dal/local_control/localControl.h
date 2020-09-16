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


#ifndef DAL_LOCALCONTROL_LOCALCONTROL_H_
#define DAL_LOCALCONTROL_LOCALCONTROL_H_

#include <dal/hal.h>
#include <pidpp/PID.h>

namespace dal{
    class LocalControl{
        public:
            /// Struct for Params of PID
            struct PIDParams{
                float kp = -1.0;
                float ki = -1.0; 
                float kd = -1.0;
                float sat = 0.0;
                float wind = 0.0;
            };  
            
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------

            /// Constructor
            LocalControl();

            /// Destructor
            ~LocalControl();

            /// This method inits the PIDs.
            /// \return true if params are good or set without errors, false if something failed.
            bool init(std::vector<float> _roll, std::vector<float> _pitch, std::vector<float> _yaw, std::vector<float> _z,  std::string _typeAWU, std::vector<float> _utils);

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR CHANGE PIDS
	        //---------------------------------------------------------------------------------------------------------------------
            bool changePID(std::string _type, PIDParams _params);

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR UPDATE
            //---------------------------------------------------------------------------------------------------------------------

            /// This method set the reference position.
            /// \param _xyzYaw: desired x y z and yaw references.
            /// \param _reset: bool for reset or not the internal values of the PIDs.
            /// \return true if params are good or set without errors, false if something failed.
            bool reference(Eigen::Vector4f _xyzYaw, bool _reset);

            /// This method update the PIDs and returns the target RPY and Thrust.
            /// \param _xyzYaw: x y z and yaw parameters.
            /// \param _incT: difference between two iterations.
            /// \return values for the DJI controller.
            Eigen::Vector4f update(Eigen::Vector4f _xyzYaw, float _incT);

        private:
            /// This method update the PIDs and returns the target RPY and Thrust.
            /// \param _aX: values from PID roll.
            /// \param _aY: values from PID pitch.
            /// \param _zPush: thrust value.
            /// \return the desired values converted.
            Eigen::Vector3f accelAngleConversion(float _aX, float _aY, float _zPush);

            /// This method saturates the vale.
            /// \param _signal: signal to saturate.
            /// \param _min: min value to saturation.
            /// \param _max: max value to saturation.
            /// \return values saturated.
            float saturateSignal(float _signal, float _min, float _max);

        private:
            // Position control PIDs
            pidpp::PID *pidRoll_, *pidPitch_, *pidYaw_, *pidZ_;

            float rollLast_ = 0, pitchLast_ = 0;
            float massUAV_ = 1.9;

            float hoveringValue_ = 28;
            float maxRoll_ = 25;
            float maxPitch_ = 25;
            float maxWYaw_ = 45;
            float minThrotle_ = 0;
            float maxThrotle_ = 100;

    };
}

#endif