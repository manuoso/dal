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


#ifndef DAL_DJI_TELEMETRYDJI_H_
#define DAL_DJI_TELEMETRYDJI_H_

// Modules
#include <dal/hal.h>

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
            typedef Eigen::Matrix<float, 10, 1> VectorHardSync;
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
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type                         hardSync_FC_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_COMPASS>::type                           compass_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type             quaternion_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_VELOCITY>::type               velocity_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>::type          flightStatus_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>::type     mode_;
            DJI::OSDK::Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type                      battery_info_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA>::type      rc_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>::type       rcRaw_;
            DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>::type         controlDevice_;
            Eigen::Vector3f                                                                         localPoseGPS_;

    };
}


#endif