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
#include <signal.h>

#include <fastcom/Publisher.h>
#include <fastcom/ServiceClient.h>
#include <fastcom/ServiceServer.h>

struct xyz_data{   
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

struct RequestCommand{
    SERVICE_MESSAGE_TYPE
    int command;
};

struct ResponseError{
    SERVICE_MESSAGE_TYPE
    int error;
};


bool finish = false;

//---------------------------------------------------------------------------------------------------------------------
// Replacement handler
void finishHandler(int _sig){
    std::cout << "Finish Handler: Catch signal " << _sig << std::endl;
    finish = true;
}

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char **_argv) {

    signal(SIGINT, finishHandler);
    signal(SIGTERM, finishHandler);

    // Initialize publishers and service client
    fastcom::Publisher<xyz_data> pubPose(9001);
    fastcom::Publisher<xyz_data> pubVel(9002);

    fastcom::ServiceClient<RequestCommand, ResponseError> client("0.0.0.0", 9010);

    int mode = 0, action = 0;
	while(finish == false){
        switch(mode){
            case 0:
            {   
                std::cout << "Select action: " << std::endl;
                std::cout << "- 1: Take off" << std::endl;
                std::cout << "- 2: Land" << std::endl;
                std::cout << "- 3: Emergency Brake" << std::endl;
                std::cout << "- 4: Position" << std::endl;
                std::cout << "- 5: Velocity" << std::endl;
                std::cout << "- 9: Exit" << std::endl;
                std::cin >> action;
                if(action == 1){
                    mode = 1;
                }else if(action == 2){ 
                    mode = 2;
                }else if(action == 3){ 
                    mode = 3;
                }else if(action == 4){                 
                    mode = 4;
                }else if(action == 5){                 
                    mode = 5;
                }else if(action == 9){                 
                    mode = 9;                
                }else{
                    std::cout << "Unrecognized action, try again." << std::endl;

                }
                break;
            }
            case 1:
            {   
                RequestCommand req;
                req.command = 1;
                client.call(req);
                mode = 0;
                break;
            }
            case 2:
            {   
                RequestCommand req;
                req.command = 2;
                client.call(req);
                mode = 0;
                break;
            }
            case 3:
            {   
                RequestCommand req;
                req.command = 3;
                client.call(req);
                mode = 0;
                break;
            }
            case 4:
            {   
                float x, y, z;
                std::cout << "Enter position: " << std::endl;
                std::cout << "X: ";
                std::cin >> x;
                std::cout << "Y: ";
                std::cin >> y;
                std::cout << "Z: ";
                std::cin >> z;

                xyz_data pose;
                pose.x = x;
                pose.y = y;
                pose.z = z;
                pubPose.publish(pose);

                int fin = 0;
                std::cout << "Enter 1 to finish: ";
                std::cin >> fin;

                pose.x = 0.0;
                pose.y = 0.0;
                pose.z = 0.0;
                pubPose.publish(pose);

                mode = 0;
                break;
            }
            case 5:
            {   
                float x, y, z;
                std::cout << "Enter velocity: " << std::endl;
                std::cout << "X: ";
                std::cin >> x;
                std::cout << "Y: ";
                std::cin >> y;
                std::cout << "Z: ";
                std::cin >> z;

                xyz_data vel;
                vel.x = x;
                vel.y = y;
                vel.z = z;
                pubVel.publish(vel);

                int fin = 0;
                std::cout << "Enter 1 to finish: ";
                std::cin >> fin;

                vel.x = 0.0;
                vel.y = 0.0;
                vel.z = 0.0;
                pubVel.publish(vel);

                mode = 0;
                break;
            }
            case 9:
                std::cout << "\nEXIT..." << std::endl;
                finish = true;
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    return 0;
}
