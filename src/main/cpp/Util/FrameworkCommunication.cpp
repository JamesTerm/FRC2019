/****************************** Header ******************************\
Class Name: FrameworkCommunication [Singleton]
File Name: FrameworkCommunication.cpp
Summary: Class to manage communication between the code and the custom 
         WinForms dashboard.
Project: FRC2020
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
Special thanks to pazdera on GitHub for the inspiration on Singleton mechanics
\********************************************************************/

using namespace std;

//TODO: Check for memory leaks
namespace Util{
    
    class FrameworkCommunication{
        public:
            // Static access method
            static FrameworkCommunication& getInstance(){
                static FrameworkCommunication instance;
                return instance;
            }

        private:
            FrameworkCommunication(){}

        public:
            FrameworkCommunication(FrameworkCommunication const&) = delete;
            void operator=(FrameworkCommunication const&) = delete;

    };

}