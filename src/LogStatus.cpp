///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <DAL/LogStatus.h>
#include <iostream>

using namespace std;

namespace dal{

	LogStatus *LogStatus::mSingleton = nullptr;

	//---------------------------------------------------------------------------------------------------------------------
	void LogStatus::init(const string _appName) {
		if (!mSingleton)
			mSingleton = new LogStatus(_appName);
		else
			mSingleton->warning("Someone tried to reinitialize the Log Status", true);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogStatus::close(){
		delete mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogStatus * LogStatus::get(){
		return mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogStatus::message(const std::string & _msg, const std::string & _tag, bool _useCout) {
		double timeSpan = std::chrono::duration<double>(chrono::high_resolution_clock::now() - mInitTime).count();
		std::string logLine = to_string(timeSpan) + "\t [" + _tag + "] " + _msg + "\n";
		mSecureGuard.lock();
		mLogFile << logLine;
		mLogFile.flush();
		mSecureGuard.unlock();
		if(_useCout){
			cout << logLine;
			cout.flush();
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogStatus::status(const std::string & _msg, bool _useCout){
		message(_msg, "STATUS", _useCout);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogStatus::warning(const std::string & _msg, bool _useCout){
		message(_msg, "WARNING", _useCout);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogStatus::error(const std::string & _msg, bool _useCout){
		message(_msg, "ERROR", _useCout);
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogStatus::LogStatus(const std::string _appName) {
		mLogFile.open(_appName + to_string(time(NULL))+".txt");
		mInitTime = chrono::high_resolution_clock::now();
		status("Initialized Log Status", false);
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogStatus::~LogStatus() {
		mLogFile.close();
	}

}