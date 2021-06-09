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

#include <iostream> // TODO: DELETE THIS

#include <dal/modules/IOFunctions.hpp>

namespace dal     {
namespace modules {

    // ----------------------------------------------------------------------
    IOFunctions::IOFunctions(std::shared_ptr<HAL> & _hal) 
        : functionTimeout_(1)
    {
        hal_ = _hal;
        if (hal_ != nullptr)
            started_ = true;
        else
            started_ = false;
    }

    // ----------------------------------------------------------------------
    IOFunctions::~IOFunctions()
    {
        this->stop();
    }

    // ----------------------------------------------------------------------
    void IOFunctions::stop()
    {
        if (started_)
        {
            started_ = false;
        }
    }

    // ----------------------------------------------------------------------
    bool IOFunctions::configureChannels(Channels _channels)
    {
        if (!started_)
            return false;

        // PREDEFINED CONFIG. TODO 666: PLEASE CHANGE THIS IN FUTURE
        // Parameters: initialValue - duty cycle
        //             freq         - PWM freq
        uint32_t initOnTimeUs = 1520;   // us
        uint16_t pwmFreq      = 50;     // Hz

        std::cout << "\033[32mConfiguring channels \033[m" << std::endl;
        for(const auto &ch: _channels)
        {
            ACK::ErrorCode errConfigCh = hal_->getVehicle()->mfio->config(ch.second, ch.first, initOnTimeUs, pwmFreq, functionTimeout_);
            if (ACK::getError(errConfigCh) != ACK::SUCCESS)
            {
                ACK::getErrorCodeMessage(errConfigCh, __func__);
                std::cout << "\033[31mError in Configure Channel: " << toString(ch.first) <<" \033[m" << std::endl;
                return false;
            }
        }    
        return true;
    }

    // ----------------------------------------------------------------------
    bool IOFunctions::setPWM(MFIO::CHANNEL _channel, uint32_t _value)
    {
        if (!started_)
            return false;

        std::cout << "\033[32mSet Value PWM \033[m" << std::endl;
        ACK::ErrorCode errSetValue = hal_->getVehicle()->mfio->setValue(_channel, _value, functionTimeout_);
        if (ACK::getError(errSetValue) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(errSetValue, __func__);
            std::cout << "\033[31mError in Set PWM Value \033[m" << std::endl;
            return false;
        }
        return true;
    }

}
}
