//---------------------------------------------------------------------------------------------------------------------
//  DRONE ABSTRACTION LAYER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 ViGUS University of Seville
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