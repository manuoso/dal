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

#include <thread>
#include <memory>
#include <mutex>
#include <atomic>
#include <condition_variable>

namespace dal       {
namespace common    {
namespace patterns  {

    // ----------------------------------------------------------------------
    template<class T>
    class NotCopy
    {
        protected:
            NotCopy() {}
            ~NotCopy() {}

        private: 
            NotCopy(const NotCopy &);
            NotCopy & operator = (const NotCopy &);
    };

    // ----------------------------------------------------------------------
    template <typename T>
    class Singleton
    {
        public:
            ~Singleton();
            static T* instance()
            {  
                static std::shared_ptr<Singleton<T> > instance = nullptr;
    
                if (!instance)
                    instance.reset(new T());

                return instance.get();
            };

        protected:
            Singleton(void);

        private:
            Singleton(const Singleton &) = delete;
            Singleton(Singleton &&) = delete;
            Singleton& operator = (const Singleton&) = delete;
            Singleton& operator = (Singleton&&) = delete;
    };

    // ----------------------------------------------------------------------
    class Builder
    {
        public:
            virtual ~Builder()
            {
                // Left in blank intentionally
            }

            virtual bool produceMinimal() = 0;

            virtual bool produceAll() = 0;
    };

    // ----------------------------------------------------------------------
    class Director
    {
        public:
            ~Director()
            {
                delete builder_;
            }

            void setBuilder(Builder *_builder)
            {
                if (this->builder_)
                    this->builder_ = nullptr;

                this->builder_ = _builder;
            }

            bool buildMinimal()
            {
                return this->builder_->produceMinimal();
            }

            bool buildAll()
            {
                return this->builder_->produceAll();
            }

        private:
            Builder* builder_;
    };

}
}
}
