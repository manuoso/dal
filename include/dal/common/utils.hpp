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

#include <string>
#include <sstream>
#include <vector>
#include <math.h>

#include <dal/common/types.hpp>

namespace dal    {
namespace common {
namespace utils  {

    using namespace types;
    
    using namespace DJI::OSDK::Telemetry;

    // ----------------------------------------------------------------------
    #define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
    #define RAD2DEG(RAD) ((RAD) * ((180.0) / (C_PI)))

    // ----------------------------------------------------------------------
    inline void localPoseFromGps(std::vector<float>& _delta, void* _target, void* _origin)
    {
        GPSFused* subscriptionTarget = (GPSFused*)_target;
        GPSFused*  subscriptionOrigin = (GPSFused*)_origin;
        
        double t_lon = subscriptionTarget->longitude * 180.0 / C_PI;
        double r_lon = subscriptionOrigin->longitude * 180.0 / C_PI;
        
        double t_lat = subscriptionTarget->latitude * 180.0 / C_PI;
        double r_lat = subscriptionOrigin->latitude * 180.0 / C_PI;

        double deltaLon   = t_lon - r_lon;
        double deltaLat   = t_lat - r_lat;

        // NEU -> North East Up
        _delta[0] = DEG2RAD(deltaLon) * C_EARTH * cos(DEG2RAD(t_lat));
        _delta[1] = DEG2RAD(deltaLat) * C_EARTH;
        _delta[2] = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }

    // ----------------------------------------------------------------------
    template <class T>
    inline std::string toString(const T& _convert)
    {
        std::stringstream ss;
        ss << _convert;
        return ss.str();
    }

}
}
}
