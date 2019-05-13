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


#ifndef DAL_BACKENDS_BACKENDPX4_H_
#define DAL_BACKENDS_BACKENDPX4_H_

#include <dal/backends/Backend.h>  

// Logs
#include <dal/LogStatus.h>
#include <dal/LogTelemetry.h>

namespace dal{
    class BackendPX4: public Backend{
        public:
            /// Default constructor
            BackendPX4():Backend(){}

            /// This method is the implementation of takeoff.
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool takeOff(const float _height);

            /// This method is the implementation of land.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool land();

            /// This method is the implementation of emergency brake.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool emergencyBrake();

            /// This method is for configure a desired mission given the waypoints.
            /// \param _wayPoints: vector with the GPS coordinates of each point, where x = lat, y = lon, z = alt.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool mission(std::vector<Eigen::Vector3f> _wayPoints);

            /// This method is for start a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool start_mission();

            /// This method is for pause a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool pause_mission();

            /// This method is for stop a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool stop_mission();

            /// This method is for resume a configured mission.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool resume_mission();

            /// This method is the implementation of move to position using DJI SDK not for PX4.
            virtual bool movePosition(float _x, float _y, float _z, float _yaw, float _posThreshold = 0.2, float _yawThreshold = 1.0){return true;};

            /// This method is the implementation of position control and yaw.
            /// \param _x: desired x.
            /// \param _y: desired y.
            /// \param _z: desired z.
            /// \param _yaw: desired yaw.
            /// \param _offset: calculate the position with offset or without a offset, that is local position in frame(local) or global.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool positionCtrlYaw(float _x, float _y, float _z, float _yaw, bool _offset);
	    
            /// This method is the implementation of velocity control and yaw.
            /// \param _x: desired Vx.
            /// \param _y: desired Vy.
            /// \param _z: desired Vz.
            /// \param _yaw: desired yaw rate.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate);

            /// This method is the implementation of get Telemetry data.
            /// \param _data: struct with the desired received data.
            /// \param _printData: if true print data received.
            /// \param _saveToFile: if true save data received.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool receiveTelemetry(dataTelemetry& _data, bool _printData, bool _saveToFile);

        private:
            /// This method initialize and some important params.
            /// \param _config: needed configuration to initialize.
            virtual bool init(const Config &_config);

        private:
            std::mutex mSecureGuard;
            
    };
}

#endif
