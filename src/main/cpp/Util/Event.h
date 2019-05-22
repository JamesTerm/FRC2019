/****************************** Header ******************************\
Class Name: EventHandler
File Name: Event.h
Summary: Logic used to run observer-model events throughout the codebase
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
\********************************************************************/

#pragma once

#include <assert.h>
#include <vector>
#include <functional>
#include <memory>
#include <iostream>

#include "EventArgs.h"
#include "Log.h"

using namespace std;
using namespace Logger;

static int counter;

struct EventHandlerAssignmentException : public exception
{
	const char *what() const throw()
	{
		return "An exception occured while assigning an EventHandler";
	}
};

class EventHandler
{

  public:
	int id;
	using Func = std::function<void(EventArgs *)>;

  private:
	Func _func;

#pragma region CTOR
  public:
	EventHandler() : id{0} {}
	EventHandler(const Func &func) : _func{func} { this->id = ++counter; };
#pragma endregion

#pragma region OPERATOR_OVERLOADS
  public:
	void operator()(EventArgs *e) { this->_func(e); }
	bool operator!=(nullptr_t) { return this->_func != nullptr; }
	bool operator==(const EventHandler &del) { return this->id == del.id; }

	void operator=(const EventHandler &func)
	{
		if (this->_func == nullptr)
		{
			this->_func = func;
			this->id = ++counter;
		}
		else
			throw EventHandlerAssignmentException();
	}
#pragma endregion
};

class Event
{

  private:
	bool firstHandlerSet = false;
	std::unique_ptr<EventHandler> firstHandler;
	std::vector<std::unique_ptr<EventHandler>> handlers;

	void addHandler(const EventHandler &handler)
	{
		this->handlers.push_back(unique_ptr<EventHandler>(new EventHandler{handler}));
	}

	void addHandlerFirst(const EventHandler &handler)
	{
		this->firstHandler = unique_ptr<EventHandler>(new EventHandler{handler});
	}

	void removeHandler(const EventHandler &handler)
	{
		vector<unique_ptr<EventHandler>>::iterator to_remove = this->handlers.begin();
		for (; to_remove != this->handlers.end(); ++to_remove)
			if (*(*to_remove) == handler)
			{
				this->handlers.erase(to_remove);
				break;
			}
	}

	void removeHandlerFirst()
	{
		this->firstHandler = NULL;
	}

	void notifyHandlers(EventArgs *e)
	{
		if (firstHandlerSet)
		{
			(*firstHandler)(e);
		}
		vector<unique_ptr<EventHandler>>::iterator func = this->handlers.begin();
		for (; func != this->handlers.end(); ++func)
			if (*func != nullptr && (*func)->id != 0)
				(*(*func))(e);
		delete e;
		e = nullptr;
	}

#pragma region OPERATOR_OVERLOADS
  public:
	void operator()(EventArgs *e) { this->notifyHandlers(e); }

	Event &operator+=(const EventHandler &handler)
	{
		this->addHandler(handler);
		return *this;
	}

	Event &operator+=(const EventHandler::Func &handler)
	{
		this->addHandler(EventHandler{handler});
		return *this;
	}

	Event &operator-=(const EventHandler &handler)
	{
		this->removeHandler(handler);
		return *this;
	}

	Event &operator-=(const EventHandler::Func &handler)
	{
		this->removeHandler(EventHandler{handler});
		return *this;
	}

	void subscribeFirstPriority(const EventHandler &handler)
	{
		if (firstHandlerSet)
		{
			Log::Error("Error setting the subscribeFirstPriority in Event.h! The FirstPriority Subscriber has already been set!");
			return;
		}
		addHandlerFirst(handler);
		firstHandlerSet = true;
	}

	void subscribeFirstPriority(const EventHandler::Func &handler)
	{
		if (firstHandlerSet)
		{
			Log::Error("Error setting the subscribeFirstPriority in Event.h! The FirstPriority Subscriber has already been set!");
			return;
		}
		addHandlerFirst(EventHandler{handler});
		firstHandlerSet = true;
	}

	void unsubscribeFirstPriority()
	{
		if (!firstHandlerSet)
		{
			Log::Error("Error setting the unsubscribeFirstPriority in Event.h! The FirstPriority Subscriber has not been set!");
			return;
		}
		removeHandlerFirst();
		firstHandlerSet = false;
	}
#pragma endregion
};
