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

#include <dal/hal.hpp>

namespace dal     {
namespace modules {

    class Telemetry
    {
        public:
            Telemetry(std::shared_ptr<HAL> & _hal);
            ~Telemetry();

            // ----------------------------------------------------------------------
            void stop();
            void init();

            // ----------------------------------------------------------------------
            void setCallbackGPSCoord(CallbackLatLon _cb);
            /// ----------
            void setCallbackGPSDetail(CallbackGPSDetail _cb);
            /// ----------
            void setCallbackGPSSignal(CallbackGPSSignal _cb);
            /// ----------
            void setCallbackAltitude(CallbackAltitude _cb);
            /// ----------
            void setCallbackAngRate(CallbackAngRate _cb);
            /// ----------
            void setCallbackAngRateRaw(CallbackAngRateRaw _cb);
            /// ----------
            void setCallbackAccRaw(CallbackAccRaw _cb);
            /// ----------
            void setCallbackIMU(CallbackIMU _cb);
            /// ----------
            void setCallbackCompass(CallbackCompass _cb);
            /// ----------
            void setCallbackQuat(CallbackQuat _cb);
            /// ----------
            void setCallbackVel(CallbackVel _cb);
            /// ----------
            void setCallbackFlightStatus(CallbackFlighStatus _cb);
            /// ----------
            void setCallbackMode(CallbackMode _cb);
            /// ----------
            void setCallbackBattery(CallbackBattery _cb);
            /// ----------
            void setCallbackRcBasic(CallbackRcBasic _cb);
            /// ----------
            void setCallbackRc(CallbackRc _cb);
            /// ----------
            void setCallbackRcRaw(CallbackRcRaw _cb);
            /// ----------
            void setCallbackControlDevice(CallbackControlDevice _cb);
            /// ----------
            void setCallbackLocalPoseGPS(CallbackLocalPoseGPS _cb);

            // ----------------------------------------------------------------------
            LatLon getGPSCoord();
            /// ----------
            GPSDetail getGPSDetail();
            /// ----------
            GPSSignal getGPSSignal();
            /// ----------
            Altitude getAltitude();
            /// ----------
            AngularRate getAngRate();
            /// ----------
            AngularRateRaw getAngRateRaw();
            /// ----------
            AccelerationRaw getAccRaw();
            /// ----------
            HardSync_FC getIMU();
            /// ----------
            Compass getCompass();
            /// ----------
            Quaternion getQuat();
            /// ----------
            Velocity getVel();
            /// ----------
            FlightStatus getFlightStatus();
            /// ----------
            Mode getMode();
            /// ----------
            Battery_info getBattery();
            /// ----------
            RcBasic getRcBasic();
            /// ----------
            Rc getRc();
            /// ----------
            RcRaw getRcRaw();
            /// ----------
            ControlDevice getControlDevice();
            /// ----------
            LocalPoseGPS getLocalPoseGPS();

        private:
            static void callback400Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
            static void callback200Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
            static void callback50Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
            static void callback5Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
        
        private:
            std::shared_ptr<HAL> hal_;

            std::atomic<bool> started_;
            int functionTimeout_;

            Topics topics400hz_, topics200hz_, topics50hz_, topics5hz_;

            std::mutex mtxGPSCoord_;
            std::mutex mtxGPSDetail_;
            std::mutex mtxGPSSignal_;
            std::mutex mtxAltitude_;
            std::mutex mtxAngRate_;
            std::mutex mtxAngRateRaw_;
            std::mutex mtxAccRaw_;
            std::mutex mtxIMU_;
            std::mutex mtxCompass_;
            std::mutex mtxQuat_;
            std::mutex mtxVel_;
            std::mutex mtxFlightStatus_;
            std::mutex mtxMode_;
            std::mutex mtxBattery_;
            std::mutex mtxRcBasic_;
            std::mutex mtxRc_;
            std::mutex mtxRcRaw_;
            std::mutex mtxControlDevice_;
            std::mutex mtxLocalPoseGPS_;

            LatLon          latLon_;
            GPSDetail       gpsDetail_;
            GPSSignal       gpsSignal_;
            Altitude        altitude_;
            AngularRate     angRate_;
            AngularRateRaw  angRateRaw_;
            AccelerationRaw accRaw_;
            HardSync_FC     hs_fc_;
            Compass         compass_;
            Quaternion      quat_;
            Velocity        vel_;
            FlightStatus    fs_;
            Mode            mode_;
            Battery_info    bat_info_;
            RcBasic         rcBasic_;
            Rc              rc_;
            RcRaw           rcRaw_;
            ControlDevice   contDevice_;
            LocalPoseGPS    localPoseGPS_;

            CallbackLatLon          cbLatLon_;
            CallbackGPSDetail       cbGPSDetail_;
            CallbackGPSSignal       cbGPSSignal_;
            CallbackAltitude        cbAltitude_;
            CallbackAngRate         cbAngRate_;
            CallbackAngRateRaw      cbAngRateRaw_;
            CallbackAccRaw          cbAccRaw_;
            CallbackIMU             cbIMU_;
            CallbackCompass         cbCompass_;
            CallbackQuat            cbQuat_;
            CallbackVel             cbVel_;
            CallbackFlighStatus     cbFlightStatus_;
            CallbackMode            cbMode_;
            CallbackBattery         cbBattery_;
            CallbackRcBasic         cbRcBasic_;
            CallbackRc              cbRc_;
            CallbackRcRaw           cbRcRaw_;
            CallbackControlDevice   cbControlDevice_;
            CallbackLocalPoseGPS    cbLocalPoseGPS_;

    };
}
}

/*
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// ¡¡¡ IMPORTANT !!! 
//
// This backend is developed for the DJI A3 controller. 
// So the implemented functions may vary for another model like the M210 and M600.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

namespace dal{
    class TelemetryDJI{
        public:
            /// Especials Typedefs
            typedef Eigen::Matrix<float, 6, 1> VectorPositionVO;
            typedef Eigen::Matrix<float, 2, 1> VectorGPS;
            typedef Eigen::Matrix<float, 3, 1> VectorGPSRaw;
            typedef Eigen::Matrix<float, 8, 1> VectorGPSDetail;
            typedef Eigen::Matrix<float, 3, 1> VectorAngularRate;
            typedef Eigen::Matrix<float, 3, 1> VectorAngularRateRaw; 
            typedef Eigen::Matrix<float, 3, 1> VectorAccelerationRaw;      
            typedef Eigen::Matrix<float, 10, 1> VectorHardSync;
            typedef Eigen::Matrix<float, 6, 1> VectorBasicRC;
            typedef Eigen::Matrix<float, 8, 1> VectorRC;
            typedef Eigen::Matrix<float, 21, 1> VectorRCRaw;
            typedef Eigen::Matrix<float, 3, 1> VectorCompass;
            typedef Eigen::Matrix<float, 4, 1> VectorQuaternion;
            typedef Eigen::Matrix<float, 3, 1> VectorVelocity;
            typedef Eigen::Matrix<int, 3, 1> VectorControlDevice;
            typedef Eigen::Matrix<float, 3, 1> VectorLocalPositionGPS;

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR INITIALIZATION
	        //---------------------------------------------------------------------------------------------------------------------

            /// Constructor
            TelemetryDJI();

            /// Destructor
            ~TelemetryDJI();

            //---------------------------------------------------------------------------------------------------------------------
            // METHODS FOR TELEMETRY
	        //---------------------------------------------------------------------------------------------------------------------

            /// This method is the implementation of get local position VO.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getPositionVO(VectorPositionVO& _data);

            /// This method is the implementation of get GPS.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getGPS(VectorGPS& _data);

            /// This method is the implementation of get GPS raw data.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getGPSRaw(VectorGPSRaw& _data);

            /// This method is the implementation of get GPS Detail.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getGPSDetail(VectorGPSDetail& _data);

            /// This method is the implementation of get GPS Signal Level.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getGPSSignal(int& _data);

            /// This method is the implementation of get Altitude fusioned.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getAltitude(float& _data);

            /// This method is the implementation of get Angular Rate fusioned.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getAngularRate(VectorAngularRate& _data);

            /// This method is the implementation of get Angular Rate raw from IMU.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getAngularRateRaw(VectorAngularRateRaw& _data);

            /// This method is the implementation of get Acceleration raw from IMU.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getAccelerationRaw(VectorAccelerationRaw& _data);

            /// This method is the implementation of get Hard Sync.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getHardSync(VectorHardSync& _data);

            /// This method is the implementation of get Compass.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getCompass(VectorCompass& _data);

            /// This method is the implementation of get Quaternion.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getQuaternion(VectorQuaternion& _data);

            /// This method is the implementation of get Velocity.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getVelocity(VectorVelocity& _data);

            /// This method is the implementation of get Status Flight.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getStatusFlight(std::string& _data);

            /// This method is the implementation of get Display Mode.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getDisplayMode(std::string& _data);

            /// This method is the implementation of get Batery Info.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getBatery(int& _data);
            
            /// This method is the implementation of get RC without any flag data, only basic data.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getBasicRC(VectorBasicRC& _data);

            /// This method is the implementation of get RC with Flag Data.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getRC(VectorRC& _data);

            /// This method is the implementation of get raw remote controller stick, buttons and switch data
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getRCRaw(VectorRCRaw& _data);

            /// This method is the implementation of get Control Device Info.
            /// \param _data: struct with the desired received data.
            /// \return true if params are good or set without errors, false if something failed.
            bool getControlDevice(VectorControlDevice& _data);

            /// This method is the implementation of get local position using GPS.
            /// \param _data: x, y and z returned (m).
            /// \return true if params are good or set without errors, false if something failed.
            bool getLocalPositionGPS(VectorLocalPositionGPS& _data);
        
        private:
            /// This method is the implementation of a very simple calculation of local NED offset between two pairs of GPS coordinates. Accurate when distances are small.
            /// \param _delta: offset returned in NED coordinates.
            /// \param _target: target position in GPS coordinates.
            /// \param _origin: origin position in GPS coordinates.
            void localPoseFromGps(Eigen::Vector3f& _delta, void* _target, void* _origin);

        private:            
            // Data for internal telemetry
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_POSITION_VO>::type            position_vo_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>::type              latLon_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_POSITION>::type           rawLatLonAlt_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS>::type            GPSDetail_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL>::type       GPSSignal_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>::type      altitude_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type  angularRate_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_RAW>::type       angularRateRaw_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ACCELERATION_RAW>::type       accelerationRaw_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type                         hardSync_FC_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_COMPASS>::type                           compass_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type             quaternion_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_VELOCITY>::type               velocity_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>::type          flightStatus_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>::type     mode_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type                      battery_info_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC>::type                     rcBasic_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA>::type      rc_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>::type       rcRaw_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>::type         controlDevice_;
            Eigen::Vector3f                                                                         localPoseGPS_;

    };
}

*/