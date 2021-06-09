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


#pragma once

#include <functional>
#include <future>

#include <dal/hal.hpp>

namespace dal     {
namespace modules {

    class Control
    {
        public:
            Control(std::shared_ptr<HAL> & _hal);
            ~Control();
            
            // ----------------------------------------------------------------------
            void stop();
            
            // ----------------------------------------------------------------------
            bool recoverFromManual();
            bool releaseAuthority();

            // ----------------------------------------------------------------------
            void emergencyBrake();
            bool arm();
            bool disarm();

            // ----------------------------------------------------------------------
            bool takeOff(bool _block);
            bool land(bool _block);

            // ----------------------------------------------------------------------
            /// \param _x: desired x in NEU coordinates (m).
            /// \param _y: desired y in NEU coordinates (m).
            /// \param _z: desired z in NEU coordinates (m).
            /// \param _yaw: desired yaw (deg).
            bool position(float _x, float _y, float _z, float _yaw);

            /// \param _x: desired Vx in NEU coordinates (m/s).
            /// \param _y: desired Vy in NEU coordinates (m/s).
            /// \param _z: desired Vz in NEU coordinates (m/s).
            /// \param _yaw: desired yaw rate (deg/s).
            bool velocity(float _vx, float _vy, float _vz, float _yawRate);

            /// \param _roll: attitude set-point in x axis of body frame in FRU coordinates (deg).
            /// \param _pitch: attitude set-point in y axis of body frame in FRU coordinates (deg).
            /// \param _yawRate: attitude rate set-point in z axis of body frame FRU coordinates (deg/s).
            /// \param _thrust: 0-100 value for altitude stabilization.
            bool rpyThrust(float _roll, float _pitch, float _yawRate, float _thrust);

            bool customControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP);

        private:
            void launchTakeoff(std::function<void(int)> _cb);
            void launchLand(std::function<void(int)> _cb);

        private:
            std::shared_ptr<HAL> hal_;

            std::atomic<bool> started_;
            int functionTimeout_;
            std::atomic<bool> controlAct_;

            std::condition_variable condVar_;
    };
    
}
}
