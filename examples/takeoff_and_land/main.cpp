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

#include <dal/dal.h>
#include <dal/hal.h>
#include <chrono>
#include <thread>
#include <iostream>

using std::this_thread::sleep_for;
using namespace std::chrono;
using namespace std;

int main(int _argc, char ** _argv){

    if(_argc != 5){
        cout    <<  "Bad input arguments. Usage: " <<
                    "\t ./takeoff_and_land [app_id] [app_key] [baudrate] [/dev/ttyXXXX]" << endl;
    }

    dal::HAL::Config config;
    config.app_id   = atoi(_argv[1]);
    config.app_key  = _argv[2];
    config.baudrate = atoi(_argv[3]);
    config.device   = _argv[4];
    
    for(unsigned i = 0; i < 5 ; i++){
        cout << "Trying to connect...." << endl;
        
        auto autopilot = dal::DAL::create(config);
        if(autopilot){
            cout << "Connected! Sleeping 5 seconds and exiting" << endl;
            sleep_for(seconds(5));
            break;
        }
        sleep_for(seconds(1));
    }

    return -1;

}