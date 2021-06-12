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

#include <dal/hal.hpp>

namespace dal     {
namespace modules {

    using namespace DJI::OSDK;
    using namespace DJI::OSDK::Telemetry;

    class Missions
    {
        public:
            Missions(std::shared_ptr<HAL> & _hal);
            ~Missions();

            // ----------------------------------------------------------------------
            void stop();

            // ----------------------------------------------------------------------
            /// \param _wayPoint: data with the GPS coordinates, where 0 = lat, 1 = lon, 2 = alt.
            bool positionGPS(std::vector<float> _wayPoint, dataMission _config);
            
            /// \param _wayPoints: vector with the GPS coordinates of each point, where 0 = lat, 1 = lon, 2 = alt.
            /// If you select hotpoint, _wayPoints[0][0] will be the longitude, 
            /// _wayPoints[0][1] will be the latitude and _wayPoints[0][2] the altitude of the hotpoint.
            bool mission(std::map<int, std::vector<float>> _wayPoints, dataMission _config);

            bool start_mission();
            bool pause_mission();
            bool stop_mission();
            bool resume_mission();

        private:
            void setWaypointDefaults(DJI::OSDK::WayPointSettings* _wp);
            void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* _fdata);
            std::vector<DJI::OSDK::WayPointSettings> createWaypoints(std::map<int, std::vector<float>> _wayPoints, dataMission _config);
            void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& _wpList);

        private:
            std::shared_ptr<HAL> hal_;

            std::atomic<bool> started_;
            int functionTimeout_;

            std::string missionType_;

    };
    
}
}
