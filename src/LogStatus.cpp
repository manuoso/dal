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

#include <dal/LogStatus.h>
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
