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

#include <iostream> // TODO: DELETE THIS

#include <dal/modules/Missions.hpp>

namespace dal     {
namespace modules {

    // ----------------------------------------------------------------------
    Missions::Missions(std::shared_ptr<HAL> & _hal) 
        : functionTimeout_(1)
    {
        hal_ = _hal;
        if (hal_ != nullptr)
            started_ = true;
        else
            started_ = false;
    }

    // ----------------------------------------------------------------------
    Missions::~Missions()
    {
        this->stop();
    }

    // ----------------------------------------------------------------------
    void Missions::stop()
    {
        if (started_)
        {
            started_ = false;
        }
    }

    // ----------------------------------------------------------------------
    bool Missions::positionGPS(std::vector<float> _wayPoint, dataMission _config)
    {
        if (!started_)
            return false;

        WayPointInitSettings fdata;
        setWaypointInitDefaults(&fdata);

        fdata.maxVelocity = _config.maxVelWP;
        fdata.idleVelocity = _config.idleVelWP;
        fdata.finishAction = _config.finishActionWP;
        fdata.executiveTimes = _config.executiveTimesWP;
        fdata.yawMode = _config.yawMode;
        fdata.traceMode = _config.traceModeWP;
        fdata.RCLostAction = _config.rcLostWP;
        fdata.indexNumber = 2;

        ACK::ErrorCode initAck = hal_->getVehicle()->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, functionTimeout_, &fdata);
        if (ACK::getError(initAck))
        {
            ACK::getErrorCodeMessage(initAck, __func__);
            std::cout << "\033[31mError at init waypoint GPS, exiting \033[m" << std::endl;
            return false;
        }
        
        // Get actual Lat and Lon
        TypeMap<TOPIC_GPS_FUSED>::type gps = hal_->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();

        float altitude = hal_->getVehicle()->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
        float altitudeWP = altitude - hal_->getOriginAltitude();
        
        // Create Waypoints
        WayPointSettings wp1;
        setWaypointDefaults(&wp1);

        wp1.turnMode = _config.turnModeWP;
        wp1.index     = 0;
        wp1.latitude  = gps.latitude;
        wp1.longitude = gps.longitude;
        wp1.altitude  = altitudeWP;
        
        WayPointSettings wp2;
        setWaypointDefaults(&wp2);

        wp2.turnMode = _config.turnModeWP;
        wp2.index     = 1;
        wp2.latitude  = _wayPoint[0];
        wp2.longitude = _wayPoint[1];
        wp2.altitude  = _wayPoint[2];

        std::vector<WayPointSettings> generatedWaypts;
        generatedWaypts.push_back(wp1);
        generatedWaypts.push_back(wp2);

        // Upload Waypoints
        uploadWaypoints(generatedWaypts);

        hal_->getVehicle()->missionManager->printInfo();

        ACK::ErrorCode startAck = hal_->getVehicle()->missionManager->wpMission->start(functionTimeout_);
        if (ACK::getError(startAck))
        {
            ACK::getErrorCodeMessage(startAck, __func__);
            std::cout << "\033[31mError at start waypoint GPS, exiting \033[m" << std::endl;
            return false;
        }
        else
        {
            std::cout << "\033[32mGoing to waypoint GPS \033[m" << std::endl;
        }
        return true;
    }
            
    // ----------------------------------------------------------------------
    bool Missions::mission(std::map<int, std::vector<float>> _wayPoints, dataMission _config)
    {
        if (!started_)
            return false;

        if(_config.missionType == "waypoint")
        {
            // WAYPOINTS MISSION
            missionType_ = "waypoint";

            // Waypoint Mission : Initialization
            WayPointInitSettings fdata;
            setWaypointInitDefaults(&fdata);

            fdata.maxVelocity = _config.maxVelWP;
            fdata.idleVelocity = _config.idleVelWP;
            fdata.finishAction = _config.finishActionWP;
            fdata.executiveTimes = _config.executiveTimesWP;
            fdata.yawMode = _config.yawMode;
            fdata.traceMode = _config.traceModeWP;
            fdata.RCLostAction = _config.rcLostWP;
            fdata.indexNumber = _wayPoints.size() + 2; // We add 1 to get the aircarft back to the start and pose from init

            ACK::ErrorCode initAck = hal_->getVehicle()->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, functionTimeout_, &fdata);
            if (ACK::getError(initAck))
            {
                ACK::getErrorCodeMessage(initAck, __func__);
                std::cout << "\033[31mError at init mission manager, exiting \033[m" << std::endl;
                return false;
            }

            std::cout << "\033[32mInitializing Waypoint Mission... \033[m" << std::endl;

            // Waypoint Mission: Create Waypoints
            std::vector<WayPointSettings> generatedWaypts = createWaypoints(_wayPoints, _config);
            std::cout << "\033[32mCreating Waypoints... \033[m" << std::endl;

            // Waypoint Mission: Upload the waypoints
            uploadWaypoints(generatedWaypts);
            std::cout << "\033[32mUploading Waypoints... \033[m" << std::endl;

            hal_->getVehicle()->missionManager->printInfo();
        }
        else if(_config.missionType == "hotpoint")
        {
            // HOTPOINT MISSION
            missionType_ = "hotpoint";

            // Hotpoint Mission Initialize
            hal_->getVehicle()->missionManager->init(DJI_MISSION_TYPE::HOTPOINT, functionTimeout_, NULL);

            hal_->getVehicle()->missionManager->hpMission->setHotPoint(_wayPoints[0][1], _wayPoints[0][0], _wayPoints[0][2]);

            hal_->getVehicle()->missionManager->hpMission->setRadius(_config.radiusHP);

            hal_->getVehicle()->missionManager->printInfo();
        }
        else
        {
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    // ----------------------------------------------------------------------
    bool Missions::start_mission()
    {
        if (!started_)
            return false;

        if(missionType_ == "waypoint")
        {
            ACK::ErrorCode startAck = hal_->getVehicle()->missionManager->wpMission->start(functionTimeout_);
            if (ACK::getError(startAck))
            {
                ACK::getErrorCodeMessage(startAck, __func__);
                std::cout << "\033[31mError at start mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mStarting Waypoint Mission... \033[m" << std::endl;
            }

        }
        else if(missionType_ == "hotpoint")
        {
            ACK::ErrorCode startAck = hal_->getVehicle()->missionManager->hpMission->start(functionTimeout_);
            if (ACK::getError(startAck))
            {
                ACK::getErrorCodeMessage(startAck, __func__);
                std::cout << "\033[31mError at start mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mStarting Hotpoint Mission... \033[m" << std::endl;
            }
        }
        else
        {
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true;        
    }

    // ----------------------------------------------------------------------
    bool Missions::pause_mission()
    {
        if (!started_)
            return false;

        if(missionType_ == "waypoint")
        {
            ACK::ErrorCode pauseAck = hal_->getVehicle()->missionManager->wpMission->pause(functionTimeout_);
            if (ACK::getError(pauseAck))
            {
                ACK::getErrorCodeMessage(pauseAck, __func__);
                std::cout << "\033[31mError at pause mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mPause Waypoint Mission... \033[m" << std::endl;
            }
        }
        else if(missionType_ == "hotpoint")
        {
            ACK::ErrorCode pauseAck = hal_->getVehicle()->missionManager->hpMission->pause(functionTimeout_);
            if (ACK::getError(pauseAck))
            {
                ACK::getErrorCodeMessage(pauseAck, __func__);
                std::cout << "\033[31mError at pause mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mPause Hotpoint Mission... \033[m" << std::endl;
            }
        }
        else
        {
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true; 
    }

    // ----------------------------------------------------------------------
    bool Missions::stop_mission()
    {
        if (!started_)
            return false;

        if(missionType_ == "waypoint")
        {
            ACK::ErrorCode stopAck = hal_->getVehicle()->missionManager->wpMission->stop(functionTimeout_);
            if (ACK::getError(stopAck))
            {
                ACK::getErrorCodeMessage(stopAck, __func__);
                std::cout << "\033[31mError at stop mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mStop Waypoint Mission... \033[m" << std::endl;
            }
        }
        else if(missionType_ == "hotpoint")
        {
            ACK::ErrorCode stopAck = hal_->getVehicle()->missionManager->hpMission->stop(functionTimeout_);
            if (ACK::getError(stopAck))
            {
                ACK::getErrorCodeMessage(stopAck, __func__);
                std::cout << "\033[31mError at stop mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mStop Hotpoint Mission... \033[m" << std::endl;
            }
        }
        else
        {
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true; 
    }

    // ----------------------------------------------------------------------
    bool Missions::resume_mission()
    {
        if (!started_)
            return false;

        if(missionType_ == "waypoint")
        {
            ACK::ErrorCode resumeAck = hal_->getVehicle()->missionManager->wpMission->resume(functionTimeout_);
            if (ACK::getError(resumeAck))
            {
                ACK::getErrorCodeMessage(resumeAck, __func__);
                std::cout << "\033[31mError at resume mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mResume Waypoint Mission... \033[m" << std::endl;
            }
        }
        else if(missionType_ == "hotpoint")
        {
            ACK::ErrorCode resumeAck = hal_->getVehicle()->missionManager->hpMission->resume(functionTimeout_);
            if (ACK::getError(resumeAck))
            {
                ACK::getErrorCodeMessage(resumeAck, __func__);
                std::cout << "\033[31mError at resume mission, exiting \033[m" << std::endl;
                return false;
            }
            else
            {
                std::cout << "\033[32mResume Hotpoint Mission... \033[m" << std::endl;
            }
        }
        else
        {
            std::cout << "\033[31mUnrecognised mission type, exiting \033[m" << std::endl;
            return false;
        }
        return true; 
    }

    // ----------------------------------------------------------------------
    void Missions::setWaypointDefaults(WayPointSettings* _wp)
    {
        _wp->damping         = 0;
        _wp->yaw             = 0;
        _wp->gimbalPitch     = 0;
        _wp->turnMode        = 0;
        _wp->hasAction       = 0;
        _wp->actionTimeLimit = 100;
        _wp->actionNumber    = 0;
        _wp->actionRepeat    = 0;
        for (int i = 0; i < 16; ++i)
        {
            _wp->commandList[i]      = 0;
            _wp->commandParameter[i] = 0;
        }
    }

    // ----------------------------------------------------------------------
    void Missions::setWaypointInitDefaults(WayPointInitSettings* _fdata)
    {
        _fdata->maxVelocity    = 2;
        _fdata->idleVelocity   = 1;
        _fdata->finishAction   = 0;
        _fdata->executiveTimes = 0;
        _fdata->yawMode        = 0;
        _fdata->traceMode      = 0;
        _fdata->RCLostAction   = 0;
        _fdata->gimbalPitch    = 0;
        _fdata->latitude       = 0;
        _fdata->longitude      = 0;
        _fdata->altitude       = 0;
    }

    // ----------------------------------------------------------------------
    std::vector<WayPointSettings> Missions::createWaypoints(std::map<int, std::vector<float>> _wayPoints, dataMission _config)
    {
        // Get actual Lat and Lon
        TypeMap<TOPIC_GPS_FUSED>::type gps = hal_->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();

        // Let's create a vector to store our waypoints in
        std::vector<WayPointSettings> wp_list;

        // Init wp
        WayPointSettings init_wp;
        setWaypointDefaults(&init_wp);

        auto initWP = _wayPoints[0];
        init_wp.turnMode = _config.turnModeWP;
        init_wp.index     = 0;
        init_wp.latitude  = gps.latitude;
        init_wp.longitude = gps.longitude;
        init_wp.altitude  = initWP[2];

        wp_list.push_back(init_wp);   

        for (auto it : _wayPoints)
        {
            WayPointSettings  wp;
            setWaypointDefaults(&wp);

            wp.turnMode = _config.turnModeWP;
            wp.index     = it.first;
            wp.latitude  = it.second[0];
            wp.longitude = it.second[1];
            wp.altitude  = it.second[2];
            wp_list.push_back(wp);
        }
        return wp_list;
    }

    // ----------------------------------------------------------------------
    void Missions::uploadWaypoints(std::vector<WayPointSettings>& _wpList)
    {
        for (std::vector<WayPointSettings>::iterator wp = _wpList.begin(); wp != _wpList.end(); ++wp)
        {
            std::cout << "\033[32mWaypoint created at (LLA): \033[m" << std::to_string(wp->latitude) + "\033[32m | \033[m" + std::to_string(wp->longitude) + "\033[32m | \033[m" + std::to_string(wp->altitude) << std::endl;

            ACK::WayPointIndex wpDataACK = hal_->getVehicle()->missionManager->wpMission->uploadIndexData(&(*wp), functionTimeout_);
            if (ACK::getError(wpDataACK.ack))
            {
                ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
            }
        }
    }

}
}
