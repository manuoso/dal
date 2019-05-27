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


#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cassert>

#include <dal/dal.h>

namespace dal{
    //---------------------------------------------------------------------------------------------------------------------
    DAL::DAL(const Backend::Config &_config) {
        backend_ = Backend::create(_config);
    }

    //---------------------------------------------------------------------------------------------------------------------
    DAL::~DAL() {
        delete backend_;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::takeOff(const float _height){
        return backend_->takeOff(_height);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::land(){
        return backend_->land();
    }   

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::emergencyBrake(){
        return backend_->emergencyBrake();
    }   

    //-----------------------------------------------------------------------------------------------------------------
    bool DAL::recoverFromManual(){
        return backend_->recoverFromManual();
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::mission(std::vector<Eigen::Vector3f> _wayPoints, Backend::dataMission _config){
        return backend_->mission(_wayPoints, _config);
    }  

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::start_mission(){
        return backend_->start_mission();
    }    

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::pause_mission(){
        return backend_->pause_mission();
    } 

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::stop_mission(){
        return backend_->stop_mission();
    }     

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::resume_mission(){
        return backend_->resume_mission();
    }   

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::position(float _x, float _y, float _z, float _yaw){
        return backend_->positionCtrlYaw(_x, _y, _z, _yaw);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::velocity(float _vx, float _vy, float _vz, float _yawRate){
        return backend_->velocityCtrlYaw(_vx, _vy, _vz, _yawRate);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetry(Backend::dataTelemetry& _data, bool _saveToFile){
        return backend_->receiveTelemetry(_data, _saveToFile);
    }

}

