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


#include <DAL/backends/BackendAPM.h>

namespace dal{
    //-----------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendAPM::takeOff(float _height){

        
        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendAPM::land(){

        
        return true;        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendAPM::positionCtrlYaw(float _x, float _y, float _z, float _yaw){

        
       return true;
    }
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendAPM::velocityCtrlYaw(float _vx, float _vy, float _vz, float _yawRate){

        
       return true;
    }    

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendAPM::receiveTelemetry(dataTelemetry& _data, bool _printData, bool _saveToFile){

         
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendAPM::init(const Config &_config){

        LogStatus::init("APMStatus_" + std::to_string(time(NULL)));
        LogTelemetry::init("APMTelemetry_" + std::to_string(time(NULL)));

        return true;
    } 

}
