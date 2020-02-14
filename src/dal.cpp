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

namespace dal{
    DAL *DAL::singleton_ = nullptr;

    //-----------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------------------------------
	void DAL::init(const Backend::Config &_config) {
		if(!singleton_){
			singleton_ = new DAL(_config);
        }else{
			std::cout << "Someone tried to reinitialize the DAL system" << std::endl;
        }
	}

	//---------------------------------------------------------------------------------------------------------------------
	void DAL::close(){
		delete singleton_;
	}

	//---------------------------------------------------------------------------------------------------------------------
	DAL * DAL::get(){
		return singleton_;
	}

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR CONTROL
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::emergencyBrake(){
        return backend_->emergencyBrake();
    }   

    //-----------------------------------------------------------------------------------------------------------------
    bool DAL::recoverFromManual(){
        return backend_->recoverFromManual();
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::takeOff(const float _height){
        return backend_->takeOff(_height);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::land(){
        return backend_->land();
    }   

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::position(float _x, float _y, float _z, float _yaw){
        return backend_->positionCtrlYaw(_x, _y, _z, _yaw);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::velocity(float _vx, float _vy, float _vz, float _yawRate){
        return backend_->velocityCtrlYaw(_vx, _vy, _vz, _yawRate);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::attitude(float _roll, float _pitch, float _yaw, float _z){
        return backend_->attitudeCtrlVer(_roll, _pitch, _yaw, _z);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::attitudeRate(float _rollRate, float _pitchRate, float _yawRate, float _z){
        return backend_->angularRateCtrlVer(_rollRate, _pitchRate, _yawRate, _z);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::positionGPS(Eigen::Vector3f _wayPoint, Backend::dataMission _config){
        return backend_->positionCtrlGPS(_wayPoint, _config);
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR LOCAL CONTROL
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::initPID(std::string _type, VectorPID _x, VectorPID _y, VectorPID _z){
        // kp, ki, kd, minSat, maxSat, minWind, maxWind
        secureGuard_.lock();
        pidVX_ = new PID(_x(0), _x(1), _x(2), _x(3), _x(4), _x(5), _x(6));
        pidVY_ = new PID(_y(0), _y(1), _y(2), _y(3), _y(4), _y(5), _y(6));
        pidVZ_ = new PID(_z(0), _z(1), _z(2), _z(3), _z(4), _z(5), _z(6));

        std::cout << "KpVX: " << _x(0) << " KiVX: " << _x(1) << " KdVX: " << _x(2) << " minSatVX: " << _x(3) << " maxSatVX: " << _x(4) << " minWVX: " << _x(5) << " maxWVX: " << _x(6)  << std::endl;
        std::cout << "KpVY: " << _y(0) << " KiVY: " << _y(1) << " KdVY: " << _y(2) << " minSatVY: " << _y(3) << " maxSatVY: " << _y(4) << " minWVY: " << _y(5) << " maxWVY: " << _y(6)  << std::endl;
        std::cout << "KpVZ: " << _z(0) << " KiVZ: " << _z(1) << " KdVZ: " << _z(2) << " minSatVZ: " << _z(3) << " maxSatVZ: " << _z(4) << " minWVZ: " << _z(5) << " maxWVZ: " << _z(6)  << std::endl;
        
        if(_type == "velocity"){
            std::cout << "Using velocity control" << std::endl;            
        }else if(_type == "attitude"){
            pidAX_ = new PID(_x(7), _x(8), _x(9), _x(10), _x(11), _x(12), _x(13));
            pidAY_ = new PID(_y(7), _y(8), _y(9), _y(10), _y(11), _y(12), _y(13));

            std::cout << "KpAX: " << _x(7) << " KiAX: " << _x(8) << " KdAX: " << _x(9) << " minSatAX: " << _x(10) << " maxSatAX: " << _x(11) << " minWAX: " << _x(12) << " maxWAX: " << _x(13)  << std::endl;
            std::cout << "KpAY: " << _y(7) << " KiAY: " << _y(8) << " KdAY: " << _y(9) << " minSatAY: " << _y(10) << " maxSatAY: " << _y(11) << " minWAY: " << _y(12) << " maxWAY: " << _y(13)  << std::endl;

            std::cout << "Using attitude control" << std::endl;            
        }else{
            std::cout << "Unrecognized control" << std::endl;            
        }
        secureGuard_.unlock();

        pidType_ = _type;
        pidInitialized_ = true;

        t0_ = std::chrono::system_clock::now();

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector4f DAL::localControl(float _x, float _y, float _z, float _vx, float _vy, float _yaw){
        Eigen::Vector4f debug;
        if(pidInitialized_){
            t1_ = std::chrono::system_clock::now();
            incT_ = std::chrono::duration_cast<std::chrono::milliseconds>(t1_-t0_).count()/1000.0;
            t0_ = t1_;

            secureGuard_.lock();
            float vx = pidVX_->update(_x, incT_);
            float vy = pidVY_->update(_y, incT_);
            float vz = pidVZ_->update(_z, incT_);
            secureGuard_.unlock();

            if(pidType_ == "velocity"){
                // NOT WORKING WITH YAW RATE
                float yawRate = 0.0;
                backend_->velocityCtrlYaw(vx, vy, vz, yawRate);

                debug(0) = vx;
                debug(1) = vy;
                debug(2) = vz;
                debug(3) = yawRate;
            }else if(pidType_ == "attitude"){
                // WORKING WITH FIXED YAW
                secureGuard_.lock();
                pidAX_->reference(vx);
                pidAY_->reference(vy);

                float ax = pidAX_->update(_vx, incT_);
                float ay = pidAY_->update(_vy, incT_);
                secureGuard_.unlock();

                Eigen::Vector4f atti = convertAttiCommands(ax, ay, refz_, _yaw);

                backend_->attitudeCtrlVer(atti(0), atti(1), atti(2), atti(3));  

                debug(0) = atti(0);
                debug(1) = atti(1);
                debug(2) = atti(2);
                debug(3) = atti(3);
            }else{
                std::cout << "Unrecognized pid type" << std::endl;
            }      
        }else{
            std::cout << "PID not initialized." << std::endl;

            debug(0) = 0.0;
            debug(1) = 0.0;
            debug(2) = 0.0;
            debug(3) = 0.0;
        }

        return debug;
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::setReferencePIDV(float _x, float _y, float _z){
        if(pidInitialized_){
            secureGuard_.lock();
            pidVX_->reference(_x);
            pidVY_->reference(_y);
            pidVZ_->reference(_z);
            refz_ = _z;
            secureGuard_.unlock();

            return true;
        }else{
            std::cout << "PID not initialized." << std::endl;
            return false;
        }

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::setKpPID(float _kp, std::string _pid){
        if(_pid == "vx"){
            secureGuard_.lock();
            pidVX_->kp(_kp);
            secureGuard_.unlock();
        }else if(_pid == "vy"){
            secureGuard_.lock();
            pidVY_->kp(_kp);
            secureGuard_.unlock();
        }else if(_pid == "vz"){
            secureGuard_.lock();
            pidVZ_->kp(_kp);
            secureGuard_.unlock();
        }else if(_pid == "ax"){
            secureGuard_.lock();
            pidAX_->kp(_kp);
            secureGuard_.unlock();
        }else if(_pid == "ay"){
            secureGuard_.lock();
            pidAY_->kp(_kp);
            secureGuard_.unlock();
        }else{
            std::cout << "Unrecognized pid" << std::endl;
            return false;
        }

        return true;

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::setKiPID(float _ki, std::string _pid){
        if(_pid == "vx"){
            secureGuard_.lock();
            pidVX_->ki(_ki);
            secureGuard_.unlock();
        }else if(_pid == "vy"){
            secureGuard_.lock();
            pidVY_->ki(_ki);
            secureGuard_.unlock();
        }else if(_pid == "vz"){
            secureGuard_.lock();
            pidVZ_->ki(_ki);
            secureGuard_.unlock();
        }else if(_pid == "ax"){
            secureGuard_.lock();
            pidAX_->ki(_ki);
            secureGuard_.unlock();
        }else if(_pid == "ay"){
            secureGuard_.lock();
            pidAY_->ki(_ki);
            secureGuard_.unlock();
        }else{
            std::cout << "Unrecognized pid" << std::endl;
            return false;
        }

        return true;

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::setKdPID(float _kd, std::string _pid){
        if(_pid == "vx"){
            secureGuard_.lock();
            pidVX_->kd(_kd);
            secureGuard_.unlock();
        }else if(_pid == "vy"){
            secureGuard_.lock();
            pidVY_->kd(_kd);
            secureGuard_.unlock();
        }else if(_pid == "vz"){
            secureGuard_.lock();
            pidVZ_->kd(_kd);
            secureGuard_.unlock();
        }else if(_pid == "ax"){
            secureGuard_.lock();
            pidAX_->kd(_kd);
            secureGuard_.unlock();
        }else if(_pid == "ay"){
            secureGuard_.lock();
            pidAY_->kd(_kd);
            secureGuard_.unlock();
        }else{
            std::cout << "Unrecognized pid" << std::endl;
            return false;
        }

        return true;

    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector4f DAL::convertAttiCommands(float _ax, float _ay, float _z, float _yaw){

        float yaw = _yaw;
        float pitch = -1*( (cos(yaw)*_ax) + (sin(yaw)*_ay) );
        float roll= ( (cos(yaw)*_ay) - (sin(yaw)*_ax) );

        // SATURATIONS
        if(roll > 0.35){
            roll = 0.35;
        }else if(roll < -0.35){
            roll = -0.35;
        }

        if(pitch > 0.35){
            pitch = 0.35;
        }else if(pitch < -0.35){
            pitch = -0.35;
        }

        Eigen::Vector4f atti;
        atti(0) = RAD2DEG(roll);
        atti(1) = RAD2DEG(pitch);
        atti(2) = RAD2DEG(yaw);
        atti(3) = _z;
        
        return atti;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR MISSIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::mission(std::vector<Eigen::Vector3f> _wayPoints, Backend::dataMission _config){
        return backend_->mission(_wayPoints, _config);
    }  

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::start_mission(){
        return backend_->start_mission();
    }    

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::pause_mission(){
        return backend_->pause_mission();
    } 

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::stop_mission(){
        return backend_->stop_mission();
    }     

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::resume_mission(){
        return backend_->resume_mission();
    }   

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR TELEMETRY
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    // INITS

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::initializeLocalPosition(){
        return backend_->initLocalPosition();
    }  

    //---------------------------------------------------------------------------------------------------------------------
    // GETTERS

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryPositionVO(Backend::VectorPositionVO& _data){
        return backend_->getPositionVO(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryGPS(Backend::VectorGPS& _data){
        return backend_->getGPS(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryGPSDetail(Backend::VectorGPSDetail& _data){
        return backend_->getGPSDetail(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryGPSSignal(int& _data){
        return backend_->getGPSSignal(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryAltitude(float& _data){
        return backend_->getAltitude(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryAngularRate(Backend::VectorAngularRate& _data){
        return backend_->getAngularRate(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryHardSync(Backend::VectorHardSync& _data){
        return backend_->getHardSync(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryCompass(Backend::VectorCompass& _data){
        return backend_->getCompass(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryQuaternion(Backend::VectorQuaternion& _data){
        return backend_->getQuaternion(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryVelocity(Backend::VectorVelocity& _data){
        return backend_->getVelocity(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryStatusFlight(std::string& _data){
        return backend_->getStatusFlight(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryDisplayMode(std::string& _data){
        return backend_->getDisplayMode(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryBatery(int& _data){
        return backend_->getBatery(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryRC(Backend::VectorRC& _data){
        return backend_->getRC(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::telemetryControlDevice(Backend::VectorControlDevice& _data){
        return backend_->getControlDevice(_data);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::localPosition(Backend::VectorLocalPosition& _data){
        return backend_->getLocalPosition(_data);
    }

    //-----------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //-----------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    DAL::DAL(const Backend::Config &_config) {
        backend_ = Backend::create(_config);
    }

    //---------------------------------------------------------------------------------------------------------------------
    DAL::~DAL() {
        delete backend_;
    }

}

