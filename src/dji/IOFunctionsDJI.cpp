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

#include <dal/dji/IOFunctionsDJI.h>

namespace dal{
    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    IOFunctionsDJI::IOFunctionsDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    IOFunctionsDJI::~IOFunctionsDJI(){}


    //---------------------------------------------------------------------------------------------------------------------
    bool IOFunctionsDJI::configureChannels(std::map<DJI::OSDK::MFIO::CHANNEL, DJI::OSDK::MFIO::MODE> _channels){
        // PREDEFINED CONFIG. TODO 666: PLEASE CHANGE THIS IN FUTURE
        // Parameters: initialValue - duty cycle
        //             freq         - PWM freq
        uint32_t initOnTimeUs = 1520;   // us
        uint16_t pwmFreq      = 50;     // Hz

        std::cout << "\033[32mConfiguring channels \033[m" << std::endl;
        for(const auto &ch: _channels){
            HAL::vehicle_->mfio->config(ch.second, ch.first, initOnTimeUs, pwmFreq, HAL::functionTimeout_);
        }    

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR OUTPUT
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    bool IOFunctionsDJI::setPWM(DJI::OSDK::MFIO::CHANNEL _channel, uint32_t _value){
        std::cout << "\033[32mSet Value PWM \033[m" << std::endl;
        HAL::vehicle_->mfio->setValue(_channel, _value, HAL::functionTimeout_);

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INPUT
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

}
