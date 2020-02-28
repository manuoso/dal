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
    DAL *DAL::dal_ = nullptr;

    //---------------------------------------------------------------------------------------------------------------------
    // PUBLIC FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    // METHODS FOR INITIALIZATION
    //---------------------------------------------------------------------------------------------------------------------

	//---------------------------------------------------------------------------------------------------------------------
	DAL * DAL::create(const HAL::Config &_config) {
		if(!dal_){
			dal_ = new DAL(_config);
        }else{
            std::cout << "\033[31mSomeone tried to reinitialize the DAL system \033[m" << std::endl;
        }
        return dal_;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void DAL::close(){
		delete dal_;
	}

    //---------------------------------------------------------------------------------------------------------------------
    // LOCAL CONTROL
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::initLC(LocalControl::VectorPID _roll, LocalControl::VectorPID _pitch, LocalControl::VectorPID _yaw, LocalControl::VectorPID _z, LocalControl::VectorUtils _utils){
        return lc_->init(_roll, _pitch, _yaw, _z, _utils);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void DAL::setRefLC(Eigen::Vector4f _xyzYaw){
        lc_->reference(_xyzYaw);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool DAL::updateLC(Eigen::Vector4f _xyzYaw){
        float hz = 30;
        Eigen::Vector4f rpyT = lc_->update(_xyzYaw, 1/hz);

        return control_->rpyThrust(rpyT[0], rpyT[1], rpyT[2], rpyT[3]);
    }

    //---------------------------------------------------------------------------------------------------------------------
    // UTILS
    //---------------------------------------------------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f DAL::toEulerAngle(void* _quaternionData){
    
        DJI::OSDK::Telemetry::Quaternion* quaternion = (DJI::OSDK::Telemetry::Quaternion*)_quaternionData;

        double q2sqr = quaternion->q2 * quaternion->q2;
        double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
        double t1 = +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
        double t2 = -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
        double t3 = +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
        double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

        t2 = (t2 > 1.0) ? 1.0 : t2;
        t2 = (t2 < -1.0) ? -1.0 : t2;

        Eigen::Vector3f result;
        result << asin(t2), atan2(t3, t4), atan2(t1, t0);

        return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // PRIVATE FUNCTIONS
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------
    DAL::DAL(const HAL::Config &_config) {
        hal_ = new HAL();
        hal_->create(_config);

        lc_ = new LocalControl();

        // Init modules
        control_ = new ControlDJI();
        missions_ = new MissionsDJI();
        telemetry_ = new TelemetryDJI();
    }

    //---------------------------------------------------------------------------------------------------------------------
    DAL::~DAL() {
        hal_->close();

        delete control_;
        delete missions_;
        delete telemetry_;
        delete lc_;
    }

}

