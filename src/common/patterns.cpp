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

#include <dal/common/patterns.hpp>

namespace dal       {
namespace common    {
namespace patterns  {

    Task::Runnable::Runnable(Task* _task) : task_(_task)
    {
        // Left in blank intentionally
    }

    Task::Runnable::~Runnable()
    {
        // Left in blank intentionally
    }

    void Task::Runnable::run()
    {
        while(task_->run_.load())
        {
            if (!task_->run_.load())
                break;

            auto before = std::chrono::steady_clock::now();
            {
                task_->step();
            }

            auto after = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(after - before);

            if(elapsed > std::chrono::milliseconds(0))
            {
                if(elapsed <= task_->duration_)
                {
                    usleep(task_->duration_.count() - elapsed.count());
                }
            }
            else {
                // This case could happen when the system clock changes. The default behavior will be sleep the period
                usleep(task_->duration_.count());
            }

        }
    }

    Task::Task(unsigned int _duration)
        :   run_(false)
    {
        duration_ = std::chrono::microseconds(_duration * 1000U);
        taskRunner_ = std::make_shared<Runnable>(this);
    }

    Task::~Task()
    {
        // Left blank intentionally 
    }

    void Task::setDuration(unsigned int _duration)
    {
        std::unique_lock<std::mutex> lock(lock_);
        duration_ = std::chrono::microseconds(_duration * 1000U);
    }

    void Task::run()
    {
        std::unique_lock<std::mutex> lock(lock_);
        run_.store(true);
        thread_ = std::shared_ptr<std::thread>(new std::thread(&Runnable::run, this->taskRunner_));
    }

    void Task::stop()
    {
        std::unique_lock<std::mutex> lock(lock_);
        if(run_.load())
        {
            run_.store(false);
            this->thread_->join();
        }
    }

}
}
}