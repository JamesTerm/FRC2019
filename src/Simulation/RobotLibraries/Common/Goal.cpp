#include <stddef.h>
#include <stdlib.h>
#include <list>
#include "../Base/Base_Includes.h"
#include "../Base/Misc.h"  //needed to define the declspec
#include "Goal.h"

void* Goal::operator new ( const size_t size )
{	return malloc( size );
}

void  Goal::operator delete ( void* ptr )
{	free( ptr );
}

void* Goal::operator new [] ( const size_t size )
{	return malloc( size );
}

void  Goal::operator delete [] ( void* ptr )
{	free( ptr );
}

  /***************************************************************************************************************/
 /*												CompositeGoal													*/
/***************************************************************************************************************/

void CompositeGoal::RemoveAllSubgoals()
{
	for (SubgoalList::iterator it = m_SubGoals.begin(); it!=m_SubGoals.end(); ++it)
	{
		(*it)->Terminate();
		delete *it;
	}
	m_SubGoals.clear();
}

CompositeGoal::~CompositeGoal()
{
	RemoveAllSubgoals();
}

Goal::Goal_Status CompositeGoal::ProcessSubgoals(double dTime_s)
{
	Goal_Status StatusOfSubGoals;
	//Remove all completed and failed goals from the front of the subgoal list
	while (!m_SubGoals.empty() && (m_SubGoals.front()->GetStatus()==eCompleted || m_SubGoals.front()->GetStatus()==eFailed))
	{
		m_SubGoals.front()->Terminate();
		delete m_SubGoals.front();
		m_SubGoals.pop_front();
	}
	//If any subgoals remain, process the one at the front of the list
	if (!m_SubGoals.empty())
	{
		//grab the status of the front-most subgoal
		StatusOfSubGoals = m_SubGoals.front()->Process(dTime_s);

		//we have to test for the special case where the front-most subgoal reports "completed" and the subgoal list contains additional goals.
		//When this is the case, to ensure the parent keeps processing its subgoal list, the "active" status is returned.
		if (StatusOfSubGoals == eCompleted && m_SubGoals.size() > 1)
			StatusOfSubGoals=eActive;
	}
	else
		StatusOfSubGoals=eCompleted;
	return StatusOfSubGoals;
}



  /***************************************************************************************************************/
 /*												MultitaskGoal													*/
/***************************************************************************************************************/

MultitaskGoal::MultitaskGoal(bool WaitAll) : m_WaitAll(WaitAll)
{
	m_Status=eInactive;
}

void MultitaskGoal::RemoveAllGoals()
{
	for (GoalList::iterator it = m_GoalsToProcess.begin(); it!=m_GoalsToProcess.end(); ++it)
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
	for (auto &it : m_GoalsToProcess)
	{
		(it)->Activate();
		//Make sure it didn't fail already
		if ((it)->GetStatus() == Goal::eFailed)
		{
			m_Status = eFailed;
			break;
		}
	}
}

Goal::Goal_Status MultitaskGoal::Process(double dTime_s)
{
	ActivateIfInactive();
	Goal_Status status=eFailed;
	size_t NonActiveCount=0;

	bool SuccessDetected=false;
	//To keep things simple we'll always run a complete iteration of all the goals for the given process cycle, and simply or in the success
	//detected... This way any success that happened will be reflected and dealt with below
	for (GoalList::iterator it = m_GoalsToProcess.begin(); it!=m_GoalsToProcess.end(); ++it)
	{
		status=(*it)->Process(dTime_s);
		//If any subgoal fails... bail
		if (status==eFailed)
			return eFailed;
		if (status!=eActive)
		{
			NonActiveCount++;
			SuccessDetected|=(status==eCompleted);
		}
	}

	//Either we wait until no more active goals exist, or if the wait all option is false, we can complete when the first detected successful completion
	//has occurred.  So for that... if no successful completion then it would fall back to the wait all logic and then evaluate if it failed
	const bool IsCompleted=((NonActiveCount>=m_GoalsToProcess.size()) || ((!m_WaitAll)&&(SuccessDetected)));
	if (!IsCompleted)
		m_Status=eActive;
	else
	{
		status=eCompleted;
		//Check the final status it is completed unless any goal failed
		for (GoalList::iterator it = m_GoalsToProcess.begin(); it!=m_GoalsToProcess.end(); ++it)
		{
			if ((*it)->GetStatus()==eFailed)
				status=eFailed;
		}
		m_Status=status;
	}
	return status;
}

void MultitaskGoal::Terminate()
{
	//ensure its all clean
	RemoveAllGoals();
	m_Status=eInactive; //make this inactive
}
