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

#include <dal/dal.hpp>

namespace dal {

    // ----------------------------------------------------------------------
    DAL::DAL() 
        : started_(false)
        , hal_(nullptr)
        , control_(nullptr)
        , telemetry_(nullptr)
        , missions_(nullptr)
        , io_(nullptr)
    {

    }

    // ----------------------------------------------------------------------
    DAL::~DAL()
    {
        this->stop();
    }

    // ----------------------------------------------------------------------
    bool DAL::isInit()
    {
        return started_;
    }

    // ----------------------------------------------------------------------
    void DAL::stop()
    {
        if (started_)
        {
            started_ = false;

            if (control_)
                control_->stop();

            if (telemetry_)
                telemetry_->stop();

            if (missions_)
                missions_->stop();

            if (io_)
                io_->stop();

            hal_->stop();
        }
    }

    // ----------------------------------------------------------------------
    void DAL::buildModulesAll()
    {
        if (started_)
            return;
        
        hal_ = std::shared_ptr<HAL> (new HAL());
        this->setBuilder(hal_.get());

        if (!this->buildAll())
            return;
        
        // control_ = std::unique_ptr<Control> (new Control(hal_));
        telemetry_ = std::unique_ptr<Telemetry> (new Telemetry(hal_));
        missions_ = std::unique_ptr<Missions> (new Missions(hal_));
        io_ = std::unique_ptr<IOFunctions> (new IOFunctions(hal_));

        started_ = true;
    }

    // ----------------------------------------------------------------------
    void DAL::buildModulesCustom(std::function<void(std::shared_ptr<HAL>)> _fun)
    {
        if (started_)
            return;
        
        hal_ = std::shared_ptr<HAL> (new HAL());
        this->setBuilder(hal_.get());

        if (!this->buildAll())
            return;
        
        _fun(hal_);

        started_ = true;
    }

    // ----------------------------------------------------------------------
    void DAL::buildHalAndModules(std::shared_ptr<HAL> _hal, std::function<void(std::shared_ptr<HAL>)> _fun)
    {
        if (started_)
            return;

        hal_ = _hal;

        _fun(hal_);

        started_ = true;
    }

}
