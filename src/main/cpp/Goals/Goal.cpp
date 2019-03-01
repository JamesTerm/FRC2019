/****************************** Header ******************************\
Class Name: CompositeGoal, MultitaskGoal
File Name: Goal.cpp
Summary: Standard goal files. These classes are usually extended from,
but it rare cases MultitaskGoal and CompositeGoal may not
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): James Killian, Chris Weeks
Email: chrisrweeks@aol.com
\********************************************************************/
#include <stddef.h>
#include <stdlib.h>
#include <list>
#include "Goal.h"

using namespace std;

/***************************************************************************************************************/
/*												CompositeGoal													*/
/***************************************************************************************************************/

void CompositeGoal::Activate()
{
	m_Status = eActive;
}
Goal::Goal_Status CompositeGoal::Process(double dTime)
{
	if (m_Status == eActive)
	{
		Goal_Status newStatus = eFailed;
		//remove finished goals
		while (!m_SubGoals.empty() && (m_SubGoals.front()->GetStatus() == eCompleted || m_SubGoals.front()->GetStatus() == eFailed))
		{
			m_SubGoals.pop_front();
		}
		if (!m_SubGoals.empty())
		{
			//create pointer toks next goal for simplicity
			Goal* currentGoal = m_SubGoals.front();
			//active the next goal if it isnt already
			if (currentGoal->GetStatus() == eInactive)
			{
				currentGoal->Activate();
			}
			newStatus = currentGoal->Process(dTime);

			if(newStatus == eCompleted && m_SubGoals.size() > 1)
			{
				newStatus = eActive;
			}
		}
		else
		{
			Terminate();
		}
		return newStatus;
	}
	else
	{
		return m_Status;
	}
}

void CompositeGoal::Terminate()
{
	m_Status = eInactive;
}
/***************************************************************************************************************/
/*												MultitaskGoal													*/
/***************************************************************************************************************/

MultitaskGoal::MultitaskGoal(ActiveCollection *activeCollection, bool WaitAll) : m_WaitAll(WaitAll)
{
	m_Status = eInactive;
}

void MultitaskGoal::RemoveAllGoals()
{
	for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
	{
		(*it)->Terminate();
		delete *it;
	}
	m_GoalsToProcess.clear();
}

MultitaskGoal::~MultitaskGoal()
{
	RemoveAllGoals();
}

void MultitaskGoal::Activate()
{

	for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
		(*it)->Activate();
	m_Status = eActive;
}
Goal::Goal_Status MultitaskGoal::Process(double dTime_s)
{
	ActivateIfInactive();
	Goal_Status status = eFailed;
	size_t NonActiveCount = 0;

	bool SuccessDetected = false;
	//To keep things simple we'll always run a complete iteration of all the goals for the given process cycle, and simply or in the success
	//detected... This way any success that happened will be reflected and dealt with below
	for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
	{

		status = (*it)->Process(dTime_s);
		if (status == eFailed)
		{
			m_Status = eFailed;
			return eFailed;
		}

		if (status != eActive)
		{
			NonActiveCount++;
			SuccessDetected |= (status == eCompleted);
		}
	}

	//Either we wait until no more active goals exist, or if the wait all option is false, we can complete when the first detected successful completion
	//has occurred.  So for that... if no successful completion then it would fall back to the wait all logic and then evaluate if it failed
	const bool IsCompleted = ((NonActiveCount >= m_GoalsToProcess.size()) || ((!m_WaitAll) && (SuccessDetected)));
	if (!IsCompleted)
		m_Status = eActive;
	else
	{
		status = eCompleted;
		//Check the final status it is completed unless any goal failed
		for (GoalList::iterator it = m_GoalsToProcess.begin(); it != m_GoalsToProcess.end(); ++it)
		{
			if ((*it)->GetStatus() == eFailed)
				status = eFailed;
		}
		m_Status = status;
	}
	return status;
}

void MultitaskGoal::Terminate()
{
	//ensure its all clean
	RemoveAllGoals();
	m_Status = eInactive; //make this inactive
}
