//---------------------------------------------------------------------------------------------------------------------
//  Handling node
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

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include <signal.h>

#include <fastcom/Subscriber.h>

/// Struct of xyz
struct xyz_data{   
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
}vel;

/// Struct of GPS
struct gps_data{   
    float latitude = 0.0;
    float longitude = 0.0;
    float altitude = 0.0;
}gps;

/// Struct of RC Commands
struct rc_data{   
    float rc1 = 0.0;
    float rc2 = 0.0;
    float rc3 = 0.0;
    float rc4 = 0.0;
    float flag1 = 0.0;
    float flag2 = 0.0;
    float flag3 = 0.0;
    float flag4 = 0.0;
}rc;

char mode[30], fs[30];
float bat = 0.0;

bool finish = false;


//---------------------------------------------------------------------------------------------------------------------
// Replacement handler
void finishHandler(int _sig){
    std::cout << "Finish Handler: Catch signal " << _sig << std::endl;
    finish = true;
}

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char **_argv) {

    std::mutex secureLock;

    fastcom::Subscriber<char[30]> subMode("0.0.0.0", 9020);
    fastcom::Subscriber<char[30]> subFligStatus("0.0.0.0", 9021);
    fastcom::Subscriber<rc_data> subRC("0.0.0.0", 9022);
    fastcom::Subscriber<gps_data> subPosGPS("0.0.0.0", 9023);
    fastcom::Subscriber<xyz_data> subVel("0.0.0.0", 9024);
    fastcom::Subscriber<float> subBatLevel("0.0.0.0", 9025);

    subMode.attachCallback([&](char *_data){
        secureLock.lock();
        strcpy(mode, _data);
        secureLock.unlock();
    });

    subFligStatus.attachCallback([&](char *_data){
        secureLock.lock();
        strcpy(fs, _data);
        secureLock.unlock();
    });

    subRC.attachCallback([&](rc_data &_data){
        secureLock.lock();
        rc.rc1 = _data.rc1;
        rc.rc2 = _data.rc2;
        rc.rc3 = _data.rc3;
        rc.rc4 = _data.rc4;
        rc.flag1 = _data.flag1;
        rc.flag2 = _data.flag2;
        rc.flag3 = _data.flag3;
        rc.flag4 = _data.flag4;        
        secureLock.unlock();
    });

    subPosGPS.attachCallback([&](gps_data &_data){
        secureLock.lock();
        gps.latitude = _data.latitude;
        gps.longitude = _data.longitude;
        gps.altitude = _data.altitude;
        secureLock.unlock();
    });

    subVel.attachCallback([&](xyz_data &_data){
        secureLock.lock();
        vel.x = _data.x;
        vel.y = _data.y;
        vel.z = _data.z;
        secureLock.unlock();
    });

    subBatLevel.attachCallback([&](float &_data){
        secureLock.lock();
        bat = _data;
        secureLock.unlock();
    });

	while(!finish){
        
        std::cout << "Telemetry received:" << std::endl;
        secureLock.lock();
        std::cout << "[Mode] -> " << mode << std::endl;
        std::cout << "[FS] -> " << fs << std::endl;
        std::cout << "[RC] -> " << "RC1: " << std::to_string(rc.rc1) << "RC2: " << std::to_string(rc.rc2) << "RC3: " << std::to_string(rc.rc3) << "RC4: " << std::to_string(rc.rc4) << "FLAG1: " << std::to_string(rc.flag1) << "FLAG2: " << std::to_string(rc.flag2) << "FLAG3: " << std::to_string(rc.flag3) << "FLAG4: " << std::to_string(rc.flag4) << std::endl;
        std::cout << "[GPS] -> " << "Lat: " << std::to_string(gps.latitude) << "Lon: " << std::to_string(gps.longitude) << "Alt: " << std::to_string(gps.altitude) << std::endl;
        std::cout << "[Velocity] -> " << "X: "<< std::to_string(vel.x) << "Y: " << std::to_string(vel.y) << "Z: " << std::to_string(vel.z) << std::endl;
        std::cout << "[Batery level] " << std::to_string(bat) << std::endl;
        secureLock.unlock();
        std::cout << "--------------------------------------------------" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));        
	}

    return 0;
}
