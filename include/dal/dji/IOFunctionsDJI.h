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


#ifndef DAL_DJI_IOFUNCTIONSDJI_H_
#define DAL_DJI_IOFUNCTIONSDJI_H_

// Modules
#include <dal/hal.h>

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// ¡¡¡ IMPORTANT !!! 
//
// This backend is developed for the DJI A3 controller. 
// So the implemented functions may vary for another model like the M210 and M600.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

namespace dal{
    class IOFunctionsDJI{
        public:
            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------

            /// Constructor
            IOFunctionsDJI();

            /// Destructor
            ~IOFunctionsDJI();

            /// This method configure the function channels used in GPIO using DJI SDK.
            /// \param _height: .
            /// \return true if params are good or set without errors, false if something failed.
            bool configureChannels(std::map<DJI::OSDK::MFIO::CHANNEL, DJI::OSDK::MFIO::MODE> _channels);

            /// Map config can be
            /// CHANNELS:
            // CHANNEL_0    |   SDK1
            // CHANNEL_1    |   SDK2    
            // CHANNEL_2    |   SDK3
            // CHANNEL_3    |   SDK4
            // CHANNEL_4    |   SDK5
            // CHANNEL_5    |   SDK6
            // CHANNEL_6    |   SDK7
            // CHANNEL_7    |   SDK8

            /// MODE:
            // MODE_PWM_OUT 
            // MODE_PWM_IN  
            // MODE_GPIO_OUT
            // MODE_GPIO_IN 
            // MODE_ADC    

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR OUTPUT
	        //---------------------------------------------------------------------------------------------------------------------
            
            /// This method is the implementation of set PWM using DJI MFIO SDK.
            /// \param _channel: desired channel.
            /// \param _value: desired value.
            /// \return true if params are good or set without errors, false if something failed.
            bool setPWM(DJI::OSDK::MFIO::CHANNEL _channel, uint32_t _value);

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INPUT
	        //---------------------------------------------------------------------------------------------------------------------

    };
}

#endif