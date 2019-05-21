//---------------------------------------------------------------------------------------------------------------------
//  DRONE ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 ViGUS University of Seville
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

#include <dal/backends/Backend.h>

#include <string>
#include <Eigen/Eigen>
#include <math.h>

namespace dal{
    class DAL {
    public:
        /// Construct with given configuration for backend
        DAL(const Backend::Config &_config);

        /// Method for take off
        bool takeOff(const float _height);

        /// Method for land
        bool land();
        
        /// Method for emergency brake
        bool emergencyBrake();

        /// Method for recover control
        bool recoverFromManual();
        
        /// Method for configure a desired mission given the waypoints
        bool mission(std::vector<Eigen::Vector3f> _wayPoints, float _radius, std::string _missionType);

        /// Method for start a configured mission
        bool start_mission();

        /// Method for pause a configured mission
        bool pause_mission();

        /// Method for stop a configured mission
        bool stop_mission();

        /// Method for resume a configured mission
        bool resume_mission();

	    /// Method for go to desire position using own function
        bool position(float _x, float _y, float _z, float _yaw);

	    /// Method for move with desire velocity using own function
        bool velocity(float _vx, float _vy, float _vz, float _yawRate);

        /// Method for get telemetry
        bool telemetry(Backend::dataTelemetry& _data, bool _saveToFile);

        Backend * backend(){return mBackend;};
    private:
        Backend *mBackend;
        
    };
}

#endif
