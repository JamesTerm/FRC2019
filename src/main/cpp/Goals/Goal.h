/****************************** Header ******************************\
Class Name: Goal, AtomicGoal, CompositeGoal, MultitaskGoal
File Name: Goal.h
Summary: Standard goal files. These classes are usually extended from,
but it rare cases MultitaskGoal and CompositeGoal may not
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): James Killian, Chris Weeks
Email: chrisrweeks@aol.com
\********************************************************************/
#pragma once

#include <stack>
#include <list>
#include <iostream>

#include "../Config/ActiveCollection.h"
#include "../Util/UtilityFunctions.h"
#include "../Controls/ControlItem.h"
#include "../Global.h"


using namespace std;
/// These classes reflect the design on page 385 in the "Programming Game AI by Example" book.  There are some minor changes/improvements from
/// What is written, but perhaps the biggest design decision to consider is if they need to be templated with an entity type.  I am going to 
/// need to run through some simulation code to know for sure, but for now I am going to steer away from doing that! The actual implementation
/// of some of these methods will reside in a cpp file.  Once the decision is final they will either stay in a cpp, or change to an 'hpp' file
/// using a nested include technique to work with a templated class.  Hopefully this will not be necessary.
//  [2/23/2010 James]

class Goal
{
	public:
		virtual ~Goal() {}
		///This contains initialization logic and represents the planning phase of the goal.  A Goal is able to call its activate method
		///any number of times to re-plan if the situation demands.
		virtual void Activate()=0;

		enum Goal_Status
		{
			eInactive,  //The goal is waiting to be activated
			eActive,    //The goal has been activated and will be processed each update step
			eCompleted, //The goal has completed and will be removed on the next update
			eFailed     //The goal has failed and will either re-plan or be removed on the next update
		};
		//TODO we may want to pass in the delta time slice here
		/// This is executed during the update step
		virtual Goal_Status Process(double dTime)=0;
		/// This undertakes any necessary tidying up before a goal is exited and is called just before a goal is destroyed.
		virtual void Terminate()=0;
		//bool HandleMessage()  //TODO get event equivalent
		//TODO see if AddSubgoal really needs to be at this level 
		Goal_Status GetStatus() const {return m_Status;}
		//Here is a very common call to do in the first line of a process update
		inline void ActivateIfInactive() {if (m_Status==eInactive) Activate();}
		inline void ReActivateIfFailed() {if (m_Status==eFailed) Activate();}

		// This ensures that Composite Goals can safely allocate atomic goals and let the base implementation delete them


		vector<double> data;
		bool HasData(int index) { return (index >= 0 && index < data.size()); }
		void Setdata(int index, double dataVal) { if(index >= 0 && index < data.size()) {data.at(index) = dataVal;} else {for(int i = data.size() - 1; i < index; i++) {data.push_back(0);} data.at(index) = dataVal;}}
		double GetData(int index) { return data.at(index); }
	protected:
		Goal_Status m_Status;
		//TODO see if Owner and Type are necessary
}; 

class AtomicGoal : public Goal
{
	protected:  //from Goal
		virtual void Activate()=0;
		virtual Goal_Status Process(double dTime) {return eCompleted;}
		virtual void Terminate()=0;
		//bool HandleMessage()  //TODO get event equivalent

};

class CompositeGoal : public Goal
{
	public:
	virtual void Activate();
	virtual Goal_Status Process(double dTime);
	virtual void Terminate();

	virtual void AddSubgoal(Goal* g) {m_SubGoals.push_back(g);}
	virtual void AddSubGoalFront(Goal* g) {m_SubGoals.push_front(g);}
	private:
	typedef std::list<Goal*> SubgoalList;
	SubgoalList m_SubGoals;
};

//Similar to a Composite goal where it is composed of a list of goals, but this one will process all goals simultaneously
class  MultitaskGoal : public Goal
{
	public:
		/// \param WaitAll if true the goal is active when the state of all objects are no longer active; If false it will either be the first
		/// completed goal or when all goals are no longer active (whichever comes first).  The idea is similar to WaitForMultipleObjects in win32.
		MultitaskGoal(ActiveCollection* activeCollection, bool WaitAll=true);
		~MultitaskGoal();
		///first add the goals here
		void AddGoal(Goal *g) {m_GoalsToProcess.push_back(g);}
		///Then call this to manually activate once all goals are added
		void Activate();
		Goal &AsGoal() {return *this;}
		virtual Goal_Status Process(double dTime);
		void Reset();
	protected:  //from Goal
		
		virtual void Terminate();
		void RemoveAllGoals();
	private:
		typedef std::list<Goal *> GoalList;
		GoalList m_GoalsToProcess;
		bool m_WaitAll;
};

