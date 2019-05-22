/****************************** Header ******************************\
Class Name: TEventArgs inherits EventArgs
File Name: EventArgs.h
Summary: Argument template class to be used with events
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
\********************************************************************/

#pragma once 

class EventArgs
{

};

template<class T1, class T2>
class TEventArgs : public EventArgs
{
private:
    T1 value;
    T2 sender;

public:
    T1 GetValue() { return value; }
    T2 GetSender() { return sender; }
    TEventArgs(T1 value, T2 sender) { this->value = value; this->sender = sender; }
};

template<class T1>
class SenderEventArgs : public EventArgs
{
private:
    T1 sender;

public:
    T1 GetSender() { return sender; }
    SenderEventArgs(T1 sender) { this->sender = sender; }
};