///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <dal/LogTelemetry.h>
#include <iostream>

using namespace std;

namespace dal{

	LogTelemetry *LogTelemetry::mSingleton = nullptr;

	//---------------------------------------------------------------------------------------------------------------------
	void LogTelemetry::init(const string _appName) {
		if (!mSingleton){
			mSingleton = new LogTelemetry(_appName);
		}else{
			cout << "Someone tried to reinitialize the Log Telemetry";
			cout.flush();
		}	
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogTelemetry::close(){
		delete mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogTelemetry * LogTelemetry::get(){
		return mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogTelemetry::message(const std::string & _msg, bool _useCout) {
		double timeSpan = std::chrono::duration<double>(chrono::high_resolution_clock::now() - mInitTime).count();
		std::string logLine = to_string(timeSpan) + " " + _msg + "\n";
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
	LogTelemetry::LogTelemetry(const std::string _appName) {
		mLogFile.open(_appName + to_string(time(NULL))+".txt");
		mInitTime = chrono::high_resolution_clock::now();
		cout << "Initialized Log Telemetry";
		cout.flush();
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogTelemetry::~LogTelemetry() {
		mLogFile.close();
	}

}