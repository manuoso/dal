//---------------------------------------------------------------------------------------------------------------------
//  DJI ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
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


#include <signal.h>
#include <iostream>

#include <dal/dal.hpp>

using namespace dal;

bool fin = false;

std::unique_ptr<DAL> dal_;

// Replacement handler
void finishHandler(int _sig)
{
    std::cout << "Finish Handler: Catch signal " << _sig << std::endl;
    dal_->stop();
    fin = true;
}

int main(int _argc, char **_argv)
{
    signal(SIGINT, finishHandler);
    signal(SIGTERM, finishHandler);

    dal_ = std::unique_ptr<DAL>(new DAL);

    dal_->buildModulesAll();

    // dal_->control()->takeOff(false);

    dal_->telemetry()->setCallbackBattery([](Battery_info _bat){ std::cout << "Battery Voltage: " << _bat.voltage << std::endl; });

    while (!fin)
    {
        sleep(1);
    }

    return 0;
}