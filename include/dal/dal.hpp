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

#include <functional>

#include <dal/common/utils.hpp>
#include <dal/common/patterns.hpp>

#include <dal/hal.hpp>
#include <dal/modules/IOFunctions.hpp>
#include <dal/modules/Control.hpp>
#include <dal/modules/Missions.hpp>
#include <dal/modules/Telemetry.hpp>

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// ¡¡¡ IMPORTANT !!! 
//
// This backend is developed for the DJI A3 controller. 
// So the implemented functions may vary for another model like the M210 and M600.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

namespace dal {

    using namespace common::patterns;
    using namespace common::utils;
    using namespace modules;

    class DAL : public Director , public NotCopy<DAL>
    {
        public:
            DAL();
            ~DAL();

            // ----------------------------------------------------------------------
            bool isInit();
            void stop();
            
            // ----------------------------------------------------------------------
            void buildModulesAll();
            void buildModulesCustom(std::function<void(std::shared_ptr<HAL>)> _fun);
            void buildHalAndModules(std::shared_ptr<HAL> _hal, std::function<void(std::shared_ptr<HAL>)> _fun);

            // ----------------------------------------------------------------------
            Control * control() {return control_.get();}
            Telemetry * telemetry() {return telemetry_.get();}
            Missions * missions() {return missions_.get();}
            IOFunctions * io_functions() {return io_.get();}

        private:
            std::atomic<bool> started_;

            std::shared_ptr<HAL> hal_;

            std::unique_ptr<Control> control_;
            std::unique_ptr<Telemetry> telemetry_;
            std::unique_ptr<Missions> missions_;
            std::unique_ptr<IOFunctions> io_;

    };

}
