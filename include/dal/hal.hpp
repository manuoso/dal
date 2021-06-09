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

#include <chrono>
#include <vector>
#include <map>

#include <dal/common/patterns.hpp>
#include <dal/common/utils.hpp>
#include <dal/common/types.hpp>

// DJI OSDK includes
#include <djiosdk/dji_status.hpp>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>

namespace dal {

    using namespace common::utils;
    using namespace common::patterns;
    using namespace common::types;

    using namespace DJI::OSDK;
    using namespace DJI::OSDK::Telemetry;

    class HAL : public Builder
    {
        public:
            HAL();
            ~HAL();

            // ----------------------------------------------------------------------
            void stop();

            // ----------------------------------------------------------------------
            bool produceMinimal();
            bool produceAll();
            bool produceCustom(Config _cfg, Topics _topics);

            // ----------------------------------------------------------------------
            Vehicle* getVehicle();

            float32_t getOriginAltitude() const { return originAltitude_; }
            GPSFused getOriginGPS() const { return originGPS_; }
        
        private:
            bool init(Config _cfg);
            bool extract(Config& _cfg);

            bool topicsMinimal();
            bool topicsAll();
            bool topicsCustom(Topics _topics);

            // ----------------------------------------------------------------------
            bool obtainControlAuthority();
            bool setLocalPosition();
            
            bool extractTopics(Topics _topics);
            bool subscribeToTopic(std::vector<TopicName> _topics, int _freq);
            void unsubscribeAllTopics();

            // ----------------------------------------------------------------------
            void showLogo();
            void showCfg(Config _cfg);

        private:
            std::atomic<bool> started_;
            std::mutex mtx_;
            std::atomic<int> pkgIndex_;

            Vehicle* vehicle_;
            int functionTimeout_;
            Config cfg_;

            float32_t originAltitude_;
            GPSFused originGPS_;
    };

}
