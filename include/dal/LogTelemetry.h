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


#ifndef DAL_LOGTELEMETRY_H_
#define DAL_LOGTELEMETRY_H_

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>

namespace dal{

	/// Thread safe class used as logging system. 
	class LogTelemetry {
	public:	//	Static interface.
		/// Initialize the logging system. 
		/// \param _appName: Base name used for the log file.
		/// \param _useCout: Write to cout too.
		static void init(const std::string _appName);

		/// Close the logging system. It makes sure that the log is closed properly.
		static void close();

		/// Get current instance of the logging system.
		static LogTelemetry* get();

	public:	// Public interface.
		/// Write message to the log system
		void message(const std::string &_msg, bool _useCout = false);

	private:	// Private interface.
		LogTelemetry(const std::string _appName);
		~LogTelemetry();

		static LogTelemetry *mSingleton;

		bool mUseCout = false;

		std::chrono::high_resolution_clock::time_point  mInitTime;
		std::ofstream mLogFile;
		std::mutex mSecureGuard;
	};
}

#endif
