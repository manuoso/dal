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


#ifndef DAL_H_
#define DAL_H_

#include <dal/hal.h>

namespace dal{
    class DAL {
        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------
            
            /// Create the system. 
            static DAL * create(const HAL::Config &_config);

            /// Close the system.
            static void close();

            //---------------------------------------------------------------------------------------------------------------------
            // MODULES
	        //---------------------------------------------------------------------------------------------------------------------
        
            ControlDJI * control(){return control_;}

            MissionsDJI * missions(){return missions_;}

            TelemetryDJI * telemetry(){return telemetry_;}

            LocalControl * local_control(){return lc_;}

            //---------------------------------------------------------------------------------------------------------------------
            // UTILS
	        //---------------------------------------------------------------------------------------------------------------------

            /// 666 TODO: NEED TO CHECK!!!
            /// This method convert a quaternion to Euler Angle.
            /// \param _quat: quaternion to convert.
            /// \return the converted result.
            Eigen::Vector3f toEulerAngle(Eigen::Vector4f _quat);

        private:
            /// Constructor with given configuration for backend
            DAL(const HAL::Config &_config);

            /// Destructor
            virtual ~DAL(); 

        private:
		    static DAL *dal_;

            HAL *hal_ = nullptr;

            ControlDJI *control_ = nullptr;
            MissionsDJI *missions_ = nullptr;
            TelemetryDJI *telemetry_ = nullptr;

            LocalControl *lc_ = nullptr;

    };
}

#endif
