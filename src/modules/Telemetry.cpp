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

#include <dal/modules/Telemetry.hpp>

namespace dal     {
namespace modules {

    // ----------------------------------------------------------------------
    Telemetry::Telemetry(std::shared_ptr<HAL> & _hal) 
        : started_(false)
        , functionTimeout_(1)
        , cbLatLon_(nullptr)
        , cbGPSDetail_(nullptr)
        , cbGPSSignal_(nullptr)
        , cbAltitude_(nullptr)
        , cbAngRate_(nullptr)
        , cbAngRateRaw_(nullptr)
        , cbAccRaw_(nullptr)
        , cbIMU_(nullptr)
        , cbCompass_(nullptr)
        , cbQuat_(nullptr)
        , cbVel_(nullptr)
        , cbFlightStatus_(nullptr)
        , cbMode_(nullptr)
        , cbBattery_(nullptr)
        , cbRcBasic_(nullptr)
        , cbRc_(nullptr)
        , cbRcRaw_(nullptr)
        , cbControlDevice_(nullptr)
        , cbLocalPoseGPS_(nullptr)
    {
        hal_ = _hal;
    }

    // ----------------------------------------------------------------------
    Telemetry::~Telemetry()
    {
        this->stop();
    }

    // ----------------------------------------------------------------------
    void Telemetry::stop()
    {
        if (started_)
        {
            started_ = false;
        }
    }

    // ----------------------------------------------------------------------
    void Telemetry::init()
    {
        if (hal_ != nullptr)
            started_ = true;
        else
            started_ = false;

        if (!started_)
            return;

        topics400hz_ = hal_->getTopics400hz();
        topics200hz_ = hal_->getTopics200hz();
        topics50hz_ = hal_->getTopics50hz();
        topics5hz_ = hal_->getTopics5hz();

        //TODO: debe coincidir el pkgindex con el orden de setup de los callbacks
        if (topics400hz_.size() > 0)
            hal_->getVehicle()->subscribe->registerUserPackageUnpackCallback(0, Telemetry::callback400Hz, this);
        
        if (topics200hz_.size() > 0)
            hal_->getVehicle()->subscribe->registerUserPackageUnpackCallback(1, Telemetry::callback200Hz, this);
        
        if (topics50hz_.size() > 0)
            hal_->getVehicle()->subscribe->registerUserPackageUnpackCallback(2, Telemetry::callback50Hz, this);
        
        if (topics5hz_.size() > 0)
            hal_->getVehicle()->subscribe->registerUserPackageUnpackCallback(3, Telemetry::callback5Hz, this);
    }

    // ----------------------------------------------------------------------
    void Telemetry::setCallbackGPSCoord(CallbackLatLon _cb)
    {
        std::unique_lock<std::mutex> lock(mtxGPSCoord_);
        if (!started_)
            return;
        else
            cbLatLon_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackGPSDetail(CallbackGPSDetail _cb)
    {
        std::unique_lock<std::mutex> lock(mtxGPSDetail_);
        if (!started_)
            return;
        else
            cbGPSDetail_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackGPSSignal(CallbackGPSSignal _cb)
    {
        std::unique_lock<std::mutex> lock(mtxGPSSignal_);
        if (!started_)
            return;
        else
            cbGPSSignal_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackAltitude(CallbackAltitude _cb)
    {
        std::unique_lock<std::mutex> lock(mtxAltitude_);
        if (!started_)
            return;
        else
            cbAltitude_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackAngRate(CallbackAngRate _cb)
    {
        std::unique_lock<std::mutex> lock(mtxAngRate_);
        if (!started_)
            return;
        else
            cbAngRate_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackAngRateRaw(CallbackAngRateRaw _cb)
    {
        std::unique_lock<std::mutex> lock(mtxAngRateRaw_);
        if (!started_)
            return;
        else
            cbAngRateRaw_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackAccRaw(CallbackAccRaw _cb)
    {
        std::unique_lock<std::mutex> lock(mtxAccRaw_);
        if (!started_)
            return;
        else
            cbAccRaw_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackIMU(CallbackIMU _cb)
    {
        std::unique_lock<std::mutex> lock(mtxIMU_);
        if (!started_)
            return;
        else
            cbIMU_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackCompass(CallbackCompass _cb)
    {
        std::unique_lock<std::mutex> lock(mtxCompass_);
        if (!started_)
            return;
        else
            cbCompass_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackQuat(CallbackQuat _cb)
    {
        std::unique_lock<std::mutex> lock(mtxQuat_);
        if (!started_)
            return;
        else
            cbQuat_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackVel(CallbackVel _cb)
    {
        std::unique_lock<std::mutex> lock(mtxVel_);
        if (!started_)
            return;
        else
            cbVel_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackFlightStatus(CallbackFlighStatus _cb)
    {
        std::unique_lock<std::mutex> lock(mtxFlightStatus_);
        if (!started_)
            return;
        else
            cbFlightStatus_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackMode(CallbackMode _cb)
    {
        std::unique_lock<std::mutex> lock(mtxMode_);
        if (!started_)
            return;
        else
            cbMode_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackBattery(CallbackBattery _cb)
    {
        std::unique_lock<std::mutex> lock(mtxBattery_);
        if (!started_)
            return;
        else
            cbBattery_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackRcBasic(CallbackRcBasic _cb)
    {
        std::unique_lock<std::mutex> lock(mtxRcBasic_);
        if (!started_)
            return;
        else
            cbRcBasic_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackRc(CallbackRc _cb)
    {
        std::unique_lock<std::mutex> lock(mtxRc_);
        if (!started_)
            return;
        else
            cbRc_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackRcRaw(CallbackRcRaw _cb)
    {
        std::unique_lock<std::mutex> lock(mtxRcRaw_);
        if (!started_)
            return;
        else
            cbRcRaw_ = std::move(_cb);
    }

    /// ----------
    void Telemetry::setCallbackControlDevice(CallbackControlDevice _cb)
    {
        std::unique_lock<std::mutex> lock(mtxControlDevice_);
        if (!started_)
            return;
        else
            return;
    }

    /// ----------
    void Telemetry::setCallbackLocalPoseGPS(CallbackLocalPoseGPS _cb)
    {
        std::unique_lock<std::mutex> lock(mtxLocalPoseGPS_);
        if (!started_)
            return;
        else
            return;
    }

    // ----------------------------------------------------------------------
    LatLon Telemetry::getGPSCoord()
    {
        std::unique_lock<std::mutex> lock(mtxGPSCoord_);
        if (!started_)
            return LatLon{};
        else
            return latLon_;
    }

    /// ----------
    GPSDetail Telemetry::getGPSDetail()
    {
        std::unique_lock<std::mutex> lock(mtxGPSDetail_);
        if (!started_)
            return GPSDetail{};
        else
            return gpsDetail_;
    }

    /// ----------
    GPSSignal Telemetry::getGPSSignal()
    {
        std::unique_lock<std::mutex> lock(mtxGPSSignal_);
        if (!started_)
            return GPSSignal{};
        else
            return gpsSignal_;
    }

    /// ----------
    Altitude Telemetry::getAltitude()
    {
        std::unique_lock<std::mutex> lock(mtxAltitude_);
        if (!started_)
            return Altitude{};
        else
            return altitude_;
    }

    /// ----------
    AngularRate Telemetry::getAngRate()
    {
        std::unique_lock<std::mutex> lock(mtxAngRate_);
        if (!started_)
            return AngularRate{};
        else
            return angRate_;
    }

    /// ----------
    AngularRateRaw Telemetry::getAngRateRaw()
    {
        std::unique_lock<std::mutex> lock(mtxAngRateRaw_);
        if (!started_)
            return AngularRateRaw{};
        else
            return angRateRaw_;
    }

    /// ----------
    AccelerationRaw Telemetry::getAccRaw()
    {
        std::unique_lock<std::mutex> lock(mtxAccRaw_);
        if (!started_)
            return AccelerationRaw{};
        else
            return accRaw_;
    }

    /// ----------
    HardSync_FC Telemetry::getIMU()
    {
        std::unique_lock<std::mutex> lock(mtxIMU_);
        if (!started_)
            return HardSync_FC{};
        else
            return hs_fc_;
    }

    /// ----------
    Compass Telemetry::getCompass()
    {
        std::unique_lock<std::mutex> lock(mtxCompass_);
        if (!started_)
            return Compass{};
        else
            return compass_;
    }

    /// ----------
    Quaternion Telemetry::getQuat()
    {
        std::unique_lock<std::mutex> lock(mtxQuat_);
        if (!started_)
            return Quaternion{};
        else
            return quat_;
    }

    /// ----------
    Velocity Telemetry::getVel()
    {
        std::unique_lock<std::mutex> lock(mtxVel_);
        if (!started_)
            return Velocity{};
        else
            return vel_;
    }

    /// ----------
    FlightStatus Telemetry::getFlightStatus()
    {
        std::unique_lock<std::mutex> lock(mtxFlightStatus_);
        if (!started_)
            return FlightStatus{};
        else
            return fs_;
    }

    /// ----------
    Mode Telemetry::getMode()
    {
        std::unique_lock<std::mutex> lock(mtxMode_);
        if (!started_)
            return Mode{};
        else
            return mode_;
    }

    /// ----------
    Battery_info Telemetry::getBattery()
    {
        std::unique_lock<std::mutex> lock(mtxBattery_);
        if (!started_)
            return Battery_info{};
        else
            return bat_info_;
    }

    /// ----------
    RcBasic Telemetry::getRcBasic()
    {
        std::unique_lock<std::mutex> lock(mtxRcBasic_);
        if (!started_)
            return RcBasic{};
        else
            return rcBasic_;
    }

    /// ----------
    Rc Telemetry::getRc()
    {
        std::unique_lock<std::mutex> lock(mtxRc_);
        if (!started_)
            return Rc{};
        else
            return rc_;
    }

    /// ----------
    RcRaw Telemetry::getRcRaw()
    {
        std::unique_lock<std::mutex> lock(mtxRcRaw_);
        if (!started_)
            return RcRaw{};
        else
            return rcRaw_;
    }

    /// ----------
    ControlDevice Telemetry::getControlDevice()
    {
        std::unique_lock<std::mutex> lock(mtxControlDevice_);
        if (!started_)
            return ControlDevice{};
        else
            return contDevice_;
    }

    /// ----------
    LocalPoseGPS Telemetry::getLocalPoseGPS()
    {
        std::unique_lock<std::mutex> lock(mtxLocalPoseGPS_);
        if (!started_)
            return LocalPoseGPS{};
        else
            return localPoseGPS_;
    }

    // ----------------------------------------------------------------------
    void Telemetry::callback400Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
    {
        auto self = (Telemetry*)userData;
        
        if (self->topics400hz_.size() > 0)
            return;

        if (std::find(self->topics400hz_.begin(), self->topics400hz_.end(), TOPIC_HARD_SYNC) != self->topics400hz_.end())
        {
            std::unique_lock<std::mutex> lockIMU(self->mtxIMU_);
            self->hs_fc_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_HARD_SYNC>();
            if (self->cbIMU_)
                self->cbIMU_(self->hs_fc_);
            lockIMU.unlock();
        }

        if (std::find(self->topics400hz_.begin(), self->topics400hz_.end(), TOPIC_ANGULAR_RATE_RAW) != self->topics400hz_.end())
        {
            std::unique_lock<std::mutex> lockAngRateRaw(self->mtxAngRateRaw_);
            self->angRateRaw_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_ANGULAR_RATE_RAW>();
            if (self->cbAngRateRaw_)
                self->cbAngRateRaw_(self->angRateRaw_);
            lockAngRateRaw.unlock();
        }

        if (std::find(self->topics400hz_.begin(), self->topics400hz_.end(), TOPIC_ACCELERATION_RAW) != self->topics400hz_.end())
        {
            std::unique_lock<std::mutex> lockAccRaw(self->mtxAccRaw_);
            self->accRaw_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_ACCELERATION_RAW>();
            if (self->cbAccRaw_)
                self->cbAccRaw_(self->accRaw_);
            lockAccRaw.unlock();
        }
    }

    // ----------------------------------------------------------------------
    void Telemetry::callback200Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
    {
        auto self = (Telemetry*)userData;
        
        if (self->topics200hz_.size() > 0)
            return;

        if (std::find(self->topics200hz_.begin(), self->topics200hz_.end(), TOPIC_ANGULAR_RATE_FUSIONED) != self->topics200hz_.end())
        {
            std::unique_lock<std::mutex> lockAngRate(self->mtxAngRate_);
            self->angRate_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_ANGULAR_RATE_FUSIONED>();
            if (self->cbAngRate_)
                self->cbAngRate_(self->angRate_);
            lockAngRate.unlock();
        }

        if (std::find(self->topics200hz_.begin(), self->topics200hz_.end(), TOPIC_QUATERNION) != self->topics200hz_.end())
        {
            std::unique_lock<std::mutex> lockQuat(self->mtxQuat_);
            self->quat_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_QUATERNION>();
            if (self->cbQuat_)
                self->cbQuat_(self->quat_);
            lockQuat.unlock();
        }

        if (std::find(self->topics200hz_.begin(), self->topics200hz_.end(), TOPIC_VELOCITY) != self->topics200hz_.end())
        {
            std::unique_lock<std::mutex> lockVel(self->mtxVel_);
            self->vel_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_VELOCITY>();
            if (self->cbVel_)
                self->cbVel_(self->vel_);
            lockVel.unlock();
        }

        if (std::find(self->topics200hz_.begin(), self->topics200hz_.end(), TOPIC_ALTITUDE_FUSIONED) != self->topics200hz_.end())
        {
            std::unique_lock<std::mutex> lockAltitude(self->mtxVel_);
            self->altitude_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
            if (self->cbAltitude_)
                self->cbAltitude_(self->altitude_);
            lockAltitude.unlock();
        }
    }

    // ----------------------------------------------------------------------
    void Telemetry::callback50Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
    {
        auto self = (Telemetry*)userData;

        if (self->topics50hz_.size() > 0)
            return;

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_GPS_FUSED) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockGPSFused(self->mtxGPSCoord_);
            self->latLon_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();
            if (self->cbLatLon_)
                self->cbLatLon_(self->latLon_);

            auto origin = self->hal_->getOriginGPS();
            localPoseFromGps(self->localPoseGPS_, static_cast<void*>(&self->latLon_), static_cast<void*>(&origin));
            if (self->cbLocalPoseGPS_)
                self->cbLocalPoseGPS_(self->localPoseGPS_);
            lockGPSFused.unlock();
        }

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_COMPASS) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockCompass(self->mtxCompass_);
            self->compass_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_COMPASS>();
            if (self->cbCompass_)
                self->cbCompass_(self->compass_);
            lockCompass.unlock();
        }

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_STATUS_FLIGHT) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockStatusFlight(self->mtxGPSCoord_);
            self->fs_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
            if (self->cbFlightStatus_)
                self->cbFlightStatus_(self->fs_);
            lockStatusFlight.unlock();
        }

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_STATUS_DISPLAYMODE) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockDisplayMode(self->mtxMode_);
            self->mode_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();
            if (self->cbMode_)
                self->cbMode_(self->mode_);
            lockDisplayMode.unlock();
            }

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_RC) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockRcBasic(self->mtxRcBasic_);
            self->rcBasic_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_RC>();
            if (self->cbRcBasic_)
                self->cbRcBasic_(self->rcBasic_);
            lockRcBasic.unlock();
        }

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_RC_WITH_FLAG_DATA) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockRc(self->mtxRc_);
            self->rc_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_RC_WITH_FLAG_DATA>();
            if (self->cbRc_)
                self->cbRc_(self->rc_);
            lockRc.unlock();
        }

        if (std::find(self->topics50hz_.begin(), self->topics50hz_.end(), TOPIC_RC_FULL_RAW_DATA) != self->topics50hz_.end())
        {
            std::unique_lock<std::mutex> lockRcRaw(self->mtxRcRaw_);
            self->rcRaw_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_RC_FULL_RAW_DATA>();
            if (self->cbRcRaw_)
                self->cbRcRaw_(self->rcRaw_);
            lockRcRaw.unlock();
        }
    }

    // ----------------------------------------------------------------------
    void Telemetry::callback5Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
    {
        auto self = (Telemetry*)userData;
        
        if (self->topics5hz_.size() > 0)
            return;

        if (std::find(self->topics5hz_.begin(), self->topics5hz_.end(), TOPIC_GPS_DETAILS) != self->topics5hz_.end())
        {
            std::unique_lock<std::mutex> lockGPSDetails(self->mtxGPSDetail_);
            self->gpsDetail_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_GPS_DETAILS>();
            if (self->cbGPSDetail_)
                self->cbGPSDetail_(self->gpsDetail_);
            lockGPSDetails.unlock();
        }

        if (std::find(self->topics5hz_.begin(), self->topics5hz_.end(), TOPIC_GPS_SIGNAL_LEVEL) != self->topics5hz_.end())
        {
            std::unique_lock<std::mutex> lockGPSSignal(self->mtxGPSSignal_);
            self->gpsSignal_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_GPS_SIGNAL_LEVEL>();
            if (self->cbGPSSignal_)
                self->cbGPSSignal_(self->gpsSignal_);
            lockGPSSignal.unlock();
        }

        if (std::find(self->topics5hz_.begin(), self->topics5hz_.end(), TOPIC_BATTERY_INFO) != self->topics5hz_.end())
        {
            std::unique_lock<std::mutex> lockBattery(self->mtxBattery_);
            self->bat_info_ = self->hal_->getVehicle()->subscribe->getValue<TOPIC_BATTERY_INFO>();
            if (self->cbBattery_)
                self->cbBattery_(self->bat_info_);
            lockBattery.unlock();
        }
    }

}
}

/*
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
    bool TelemetryDJI::getGPSRaw(VectorGPSRaw& _data){

        rawLatLonAlt_ = HAL::vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_POSITION>();

        _data[0] = rawLatLonAlt_.x;
        _data[1] = rawLatLonAlt_.y;
        _data[2] = rawLatLonAlt_.z;

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
*/