#pragma once


/// These classes reflect the design on page 385 in the "Programming Game AI by Example" book.  There are some minor changes/improvements from
/// What is written, but perhaps the biggest design decision to consider is if they need to be templated with an entity type.  I am going to 
/// need to run through some simulation code to know for sure, but for now I am going to steer away from doing that! The actual implementation
/// of some of these methods will reside in a cpp file.  Once the decision is final they will either stay in a cpp, or change to an 'hpp' file
/// using a nested include technique to work with a templated class.  Hopefully this will not be necessary.
//  [2/23/2010 James]

//I removed COMMON_API from the 4 basic goal classes for right now. ~chris
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
		virtual Goal_Status Process(double dTime_s)=0;
		/// This undertakes any necessary tidying up before a goal is exited and is called just before a goal is destroyed.
		virtual void Terminate()=0;
		//bool HandleMessage()  //TODO get event equivalent
		//TODO see if AddSubgoal really needs to be at this level 
		Goal_Status GetStatus() const {return m_Status;}
		//Here is a very common call to do in the first line of a process update
		inline void ActivateIfInactive() {if (m_Status==eInactive) Activate();}
		inline void ReActivateIfFailed() {if (m_Status==eFailed) Activate();}

		// This ensures that Composite Goals can safely allocate atomic goals and let the base implementation delete them
		static void* operator new ( const size_t size );
		static void  operator delete ( void* ptr );
		static void* operator new [] ( const size_t size );
		static void  operator delete [] ( void* ptr );
	protected:
		Goal_Status m_Status;
		//TODO see if Owner and Type are necessary
}; 

class   AtomicGoal : public Goal
{
	protected:  //from Goal
		virtual void Activate() {}
		virtual Goal_Status Process(double dTime_s) {return eCompleted;}
		virtual void Terminate() {}
		//bool HandleMessage()  //TODO get event equivalent

};

class   CompositeGoal : public Goal
{
	protected:  //from Goal
		~CompositeGoal();
		virtual void Activate() {}
		virtual Goal_Status Process(double dTime_s) {return eCompleted;}
		virtual void Terminate() {}
		//bool HandleMessage()  //TODO get event equivalent
		//Subgoals are pushed in LIFO like a stack
		virtual void AddSubgoal(Goal *g) {m_SubGoals.push_front(g);}
		//Feel free to make this virtual if we find that necessary
		/// All composite goals call this method each update step to process their subgoals.  The method ensures that all completed and failed goals
		/// are removed from the list before processing the next subgoal in line and returning its status.  If the subgoal is empty eCompleted is
		/// returned.
		Goal_Status ProcessSubgoals(double dTime_s);
		void RemoveAllSubgoals();
	private:
		typedef std::list<Goal *> SubgoalList;
		SubgoalList m_SubGoals;
};

//Similar to a Composite goal where it is composed of a list of goals, but this one will process all goals simultaneously
class  MultitaskGoal : public Goal
{
	public:
		/// \param WaitAll if true the goal is active when the state of all objects are no longer active; If false it will either be the first
		/// completed goal or when all goals are no longer active (whichever comes first).  The idea is similar to WaitForMultipleObjects in win32.
		MultitaskGoal(bool WaitAll=true);
		~MultitaskGoal();
		///first add the goals here
		void AddGoal(Goal *g) {m_GoalsToProcess.push_back(g);}
		///Then call this to manually activate once all goals are added
		virtual void Activate();
		Goal &AsGoal() {return *this;}
	protected:  //from Goal
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate();
		void RemoveAllGoals();
	private:
		typedef std::list<Goal *> GoalList;
		GoalList m_GoalsToProcess;
		bool m_WaitAll;
};

//This class can be used as a stand-alone composite which does nothing special but goes through a list
class Generic_CompositeGoal : public CompositeGoal
{
	private:
		#ifndef _Win32
		typedef CompositeGoal __super;
		#endif
		bool m_AutoActivate;
	public:
		Generic_CompositeGoal(bool AutoActivate=false) : m_AutoActivate(AutoActivate)
		{
			m_Status=eInactive;
		}
		//give public access for client to populate goals
		virtual void AddSubgoal(Goal *g) {__super::AddSubgoal(g);}
		//client activates manually when goals are added
		virtual void Activate()
		{
			m_Status=eActive; 
		}

		virtual Goal_Status Process(double dTime_s)
		{
			//Client will activate if m_AutoActivate is not enabled
			if (m_AutoActivate)
				ActivateIfInactive();
			if (m_Status==eInactive)
				return m_Status;

			if (m_Status==eActive)
				m_Status=ProcessSubgoals(dTime_s);

			return m_Status;
		}
		virtual void Terminate()
		{
			//ensure its all clean
			RemoveAllSubgoals();
			m_Status=eInactive; //make this inactive
		}
};
