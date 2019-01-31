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


#ifndef DAL_BACKENDS_BACKENDAPM_H_
#define DAL_BACKENDS_BACKENDAPM_H_

// Logs
#include <dal/LogStatus.h>
#include <dal/LogTelemetry.h>

namespace dal{
    class BackendAPM{
        public:
            /// Default constructor
            BackendAPM():Backend(){}

            /// This method is the implementation of takeoff.
            /// \param _height: desired height to takeoff.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool takeOff(const float _height);

            /// This method is the implementation of land.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool land();

            /// This method is the implementation of move to position.
            /// \param _x: desired x in NEU coordinates.
            /// \param _y: desired y in NEU coordinates.
            /// \param _z: desired z in NEU coordinates.
            /// \param _yaw: desired yaw.
            /// \param _posThreshold: position threshold in the desired position in Meters.
            /// \param _yawThreshold: yaw threshold in the desired position in Deg.
            /// \return true if params are good or set without errors, false if something failed.
            virtual bool movePosition(float _x, float _y, float _z, float _yaw, float _posThreshold = 0.2, float _yawThreshold = 1.0);

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