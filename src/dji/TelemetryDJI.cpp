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

#include <dal/dji/TelemetryDJI.h>

namespace dal{
    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    TelemetryDJI::TelemetryDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    TelemetryDJI::~TelemetryDJI(){}

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR TELEMETRY
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getPositionVO(VectorPositionVO& _data){
        
        position_vo_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_POSITION_VO>();

        _data[0] = position_vo_.x;
        _data[1] = position_vo_.y;
        _data[2] = position_vo_.z;
        _data[3] = position_vo_.xHealth;
        _data[4] = position_vo_.yHealth;
        _data[5] = position_vo_.zHealth;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getGPS(VectorGPS& _data){

        latLon_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();

        _data[0] = latLon_.latitude;
        _data[1] = latLon_.longitude; 

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getGPSRaw(VectorGPSRaw& _data){

        rawLatLonAlt_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_POSITION>();

        _data[0] = rawLatLonAlt_.x;
        _data[1] = rawLatLonAlt_.y;
        _data[2] = rawLatLonAlt_.z;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getGPSDetail(VectorGPSDetail& _data){

        GPSDetail_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS>();

        _data[0] = GPSDetail_.fix;
        _data[1] = GPSDetail_.gnssStatus;
        _data[2] = GPSDetail_.hacc;
        _data[3] = GPSDetail_.sacc;
        _data[4] = GPSDetail_.usedGPS;
        _data[5] = GPSDetail_.usedGLN; 
        _data[6] = GPSDetail_.NSV;
        _data[7] = GPSDetail_.pdop;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getGPSSignal(int& _data){

        GPSSignal_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL>();

        _data = GPSSignal_;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getAltitude(float& _data){
        
        altitude_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();

        _data = altitude_ - HAL::originAltitude_;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getAngularRate(VectorAngularRate& _data){
        
        angularRate_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();

        _data[0] = angularRate_.x;
        _data[1] = angularRate_.y;
        _data[2] = angularRate_.z;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getAngularRateRaw(VectorAngularRateRaw& _data){
        
        angularRateRaw_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_RAW>();

        _data[0] = angularRateRaw_.x;
        _data[1] = angularRateRaw_.y;
        _data[2] = angularRateRaw_.z;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getAccelerationRaw(VectorAccelerationRaw& _data){
        
        accelerationRaw_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ACCELERATION_RAW>();

        _data[0] = accelerationRaw_.x;
        _data[1] = accelerationRaw_.y;
        _data[2] = accelerationRaw_.z;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getHardSync(VectorHardSync& _data){
        
        hardSync_FC_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HARD_SYNC>();

        _data[0] = hardSync_FC_.w.x;
        _data[1] = hardSync_FC_.w.y;
        _data[2] = hardSync_FC_.w.z;
        _data[3] = hardSync_FC_.a.x;
        _data[4] = hardSync_FC_.a.y;
        _data[5] = hardSync_FC_.a.z;
        _data[6] = hardSync_FC_.q.q0;
        _data[7] = hardSync_FC_.q.q1;
        _data[8] = hardSync_FC_.q.q2;
        _data[9] = hardSync_FC_.q.q3;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getCompass(VectorCompass& _data){

        compass_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_COMPASS>();

        _data[0] = compass_.x;
        _data[1] = compass_.y;
        _data[2] = compass_.z;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getQuaternion(VectorQuaternion& _data){

        quaternion_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();

        _data[0] = quaternion_.q0; 
        _data[1] = quaternion_.q1; 
        _data[2] = quaternion_.q2; 
        _data[3] = quaternion_.q3; 

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getVelocity(VectorVelocity& _data){
        
        velocity_  = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();

        _data[0] = velocity_.data.x; 
        _data[1] = velocity_.data.y; 
        _data[2] = velocity_.data.z;

        return velocity_.info.health;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getStatusFlight(std::string& _data){
        
        flightStatus_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();

        std::string sFlightStatus, sMode;
        if(flightStatus_ == 0){
            sFlightStatus = "STOPED";
        }else if(flightStatus_ == 1){
            sFlightStatus = "ON_GROUND";
        }else if(flightStatus_ == 2){
            sFlightStatus = "IN_AIR";
        }else{
            sFlightStatus = "UNRECOGNIZED";
        }

        _data = sFlightStatus; 

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getDisplayMode(std::string& _data){
        
        mode_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();

        std::string sMode;
        if(mode_ == 0){
            sMode = "MANUAL";
        }else if(mode_ == 1){
            sMode = "ATTITUDE";
        }else if(mode_ == 6){
            sMode = "P_GPS";
        }else if(mode_ == 9){
            sMode = "HOT_POINT";
        }else if(mode_ == 10){
            sMode = "ASSISTED_TAKEOFF";
        }else if(mode_ == 11){
            sMode = "AUTO_TAKEOFF";
        }else if(mode_ == 12){
            sMode = "ASSISTED_LAND";
        }else if(mode_ == 15){
            sMode = "GO_HOME";
        }else if(mode_ == 17){
            sMode = "NAVI_SDK_CTMODE_CTRLRL";
        }else if(mode_ == 33){
            sMode = "FORCE_AUTO_LANDING";
        }else if(mode_ == 40){
            sMode = "SEARCH";
        }else if(mode_ == 41){
            sMode = "ENGINE_START";
        }else{
            sMode = "UNRECOGNIZED";
        }

        _data = sMode;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getBatery(int& _data){

        battery_info_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();

        _data = battery_info_.voltage;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getBasicRC(VectorBasicRC& _data){
        rcBasic_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC>();
        
        _data[0] = rcBasic_.roll;
        _data[1] = rcBasic_.pitch;
        _data[2] = rcBasic_.yaw;
        _data[3] = rcBasic_.throttle;
        _data[4] = rcBasic_.mode;
        _data[5] = rcBasic_.gear;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getRC(VectorRC& _data){

        rc_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA>();

        _data[0] = rc_.pitch;
        _data[1] = rc_.roll;
        _data[2] = rc_.yaw;
        _data[3] = rc_.throttle;
        _data[4] = rc_.flag.logicConnected;
        _data[5] = rc_.flag.skyConnected;
        _data[6] = rc_.flag.groundConnected;
        _data[7] = rc_.flag.appConnected;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getRCRaw(VectorRCRaw& _data){
        rcRaw_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>();
        
        _data[0] = rcRaw_.lb2.roll;
        _data[1] = rcRaw_.lb2.pitch;
        _data[2] = rcRaw_.lb2.yaw;
        _data[3] = rcRaw_.lb2.throttle;
        _data[4] = rcRaw_.lb2.mode;
        _data[5] = rcRaw_.lb2.gear;
        _data[6] = rcRaw_.lb2.camera;
        _data[7] = rcRaw_.lb2.video;
        _data[8] = rcRaw_.lb2.videoPause;
        _data[9] = rcRaw_.lb2.goHome;
        _data[10] = rcRaw_.lb2.leftWheel;
        _data[11] = rcRaw_.lb2.rightWheelButton;
        _data[12] = rcRaw_.lb2.rcC1;
        _data[13] = rcRaw_.lb2.rcC2;
        _data[14] = rcRaw_.lb2.rcD1;
        _data[15] = rcRaw_.lb2.rcD2;
        _data[16] = rcRaw_.lb2.rcD3;
        _data[17] = rcRaw_.lb2.rcD4;
        _data[18] = rcRaw_.lb2.rcD5;
        _data[19] = rcRaw_.lb2.rcD6;
        _data[20] = rcRaw_.lb2.rcD7;
        _data[21] = rcRaw_.lb2.rcD8;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getControlDevice(VectorControlDevice& _data){

        controlDevice_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();

        _data[0] = controlDevice_.controlMode;
        _data[1] = controlDevice_.deviceStatus;
        _data[2] = controlDevice_.flightStatus;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool TelemetryDJI::getLocalPositionGPS(VectorLocalPositionGPS& _data){

        DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type currentSubscriptionGPS = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();

        localPoseFromGps(localPoseGPS_, static_cast<void*>(&currentSubscriptionGPS), static_cast<void*>(&HAL::originGPS_));

        _data[0] = localPoseGPS_[1];
        _data[1] = localPoseGPS_[0];
        _data[2] = localPoseGPS_[2];

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    void TelemetryDJI::localPoseFromGps(Eigen::Vector3f& _delta, void* _target, void* _origin){
    
        DJI::OSDK::Telemetry::GPSFused* subscriptionTarget = (DJI::OSDK::Telemetry::GPSFused*)_target;
        DJI::OSDK::Telemetry::GPSFused*  subscriptionOrigin = (DJI::OSDK::Telemetry::GPSFused*)_origin;
        
        double t_lon = subscriptionTarget->longitude * 180.0 / C_PI;
        double r_lon = subscriptionOrigin->longitude * 180.0 / C_PI;
        
        double t_lat = subscriptionTarget->latitude * 180.0 / C_PI;
        double r_lat = subscriptionOrigin->latitude * 180.0 / C_PI;

        double deltaLon   = t_lon - r_lon;
        double deltaLat   = t_lat - r_lat;

        // NEU -> North East Up
        _delta[0] = DEG2RAD(deltaLon) * C_EARTH * cos(DEG2RAD(t_lat));
        _delta[1] = DEG2RAD(deltaLat) * C_EARTH;
        _delta[2] = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }

}
