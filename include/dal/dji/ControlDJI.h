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


#ifndef DAL_DJI_CONTROLDJI_H_
#define DAL_DJI_CONTROLDJI_H_

// Modules
#include <dal/hal.h>

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// ¡¡¡ IMPORTANT !!! 
//
// This backend is developed for the DJI A3 controller. 
// So the implemented functions may vary for another model like the M210 and M600.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

namespace dal{
    class ControlDJI{
        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------

            /// Constructor
            ControlDJI();

            /// Destructor
            ~ControlDJI();

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR CONTROL
	        //---------------------------------------------------------------------------------------------------------------------

            /// This method is the implementation of recover control.
            /// \return true if params are good or set without errors, false if something failed.
            bool recoverFromManual();

            /// This method is the implementation of releaseAuthority.
            /// \return true if params are good or set without errors, false if something failed.
            bool releaseAuthority();

            /// This method is the implementation of emergency brake.
            /// \return true if params are good or set without errors, false if something failed.
            bool emergencyBrake();

            /// This method is the implementation of arm motors using DJI SDK, this will block the main thread.
            /// \return true if params are good or set without errors, false if something failed.
            bool arm();

            /// This method is the implementation of disarm motors using DJI SDK, this will block the main thread.
            /// \return true if params are good or set without errors, false if something failed.
            bool disarm();

            /// This method is the implementation of takeoff using DJI SDK, this will block the main thread.
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            bool takeOff(const float _height);

            /// This method is the implementation of land using DJI SDK, this will block the main thread.
            /// \return true if params are good or set without errors, false if something failed.
            bool land();

            /// This method is the implementation of position control and yaw using DJI SDK.
            /// \param _x: desired x in NEU coordinates (m).
            /// \param _y: desired y in NEU coordinates (m).
            /// \param _z: desired z in NEU coordinates (m).
            /// \param _yaw: desired yaw (deg).
            /// \return true if params are good or set without errors, false if something failed.
            bool position(float _x, float _y, float _z, float _yaw);
	    
            /// This method is the implementation of velocity control and yaw using DJI SDK.
            /// \param _x: desired Vx in NEU coordinates (m/s).
            /// \param _y: desired Vy in NEU coordinates (m/s).
            /// \param _z: desired Vz in NEU coordinates (m/s).
            /// \param _yaw: desired yaw rate (deg/s).
            /// \return true if params are good or set without errors, false if something failed.
            bool velocity(float _vx, float _vy, float _vz, float _yawRate);

            /// This method is the implementation of attitude and thrust control using custom flags from DJI SDK.
            /// \param _roll: attitude set-point in x axis of body frame in FRU coordinates (deg).
            /// \param _pitch: attitude set-point in y axis of body frame in FRU coordinates (deg).
            /// \param _yawRate: attitude rate set-point in z axis of body frame FRU coordinates (deg/s).
            /// \param _thrust: 0-100 value for altitude stabilization.
            /// \return true if params are good or set without errors, false if something failed.
            bool rpyThrust(float _roll, float _pitch, float _yawRate, float _thrust);

            /// This method is the implementation of control using DJI SDK with custom flags.
            /// \return true if params are good or set without errors, false if something failed.
            bool customControl(uint8_t _flag, float _xSP, float _ySP, float _zSP, float _yawSP);

        private:
            /// This method is the implementation of a very simple calculation of local NED offset between two pairs of GPS coordinates. Accurate when distances are small.
            /// \param _delta: offset returned in NED coordinates.
            /// \param _target: target position in GPS coordinates.
            /// \param _origin: origin position in GPS coordinates.
            void localPoseFromGps(Eigen::Vector3f& _delta, void* _target, void* _origin);

        private:
            std::atomic<bool> controlAct_;

    };
}

#endif