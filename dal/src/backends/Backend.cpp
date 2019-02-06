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


#include <dal/backends/Backend.h>
#include <dal/backends/BackendDJI.h>
#include <dal/backends/BackendAPM.h>
#include <dal/backends/BackendPX4.h>

namespace dal{
    Backend * Backend::create(const Backend::Config &_config){
        Backend *bd = nullptr;
        switch(_config.type){
        case Backend::Config::eType::DJI:
            bd = new BackendDJI();
            break;
        case Backend::Config::eType::APM:
            bd = new BackendAPM();
            break;
        case Backend::Config::eType::PX4:
            bd = new BackendPX4();
            break;
        case Backend::Config::eType::Dummy:
            bd = new BackendDummy();
            break;
        default:
            return nullptr;
        }

        if(bd->init(_config)){
            return bd;
        }else{
            return nullptr;
        }
    }

}
