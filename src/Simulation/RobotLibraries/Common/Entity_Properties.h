#pragma once

//I wanted to put this at a low enough level for common code to use... this is code that should be disabled during release
#define __DebugLUA__

class Entity1D;
class COMMON_API Entity1D_Properties
{
	public:
		Entity1D_Properties();
		Entity1D_Properties(const char EntityName[],double Mass,double Dimension,bool IsAngular=false);
		virtual ~Entity1D_Properties() {}
		virtual void LoadFromScript(Scripting::Script& script, bool NoDefaults=false);
		void Initialize(Entity1D *NewEntity) const;
		double GetMass() const {return m_Mass;}
	protected:
		std::string m_EntityName;  //derived classes can let base class know what type to read
	private:
		//Stuff needed for physics
		double m_StartingPosition;  //the position used when reset position is called
		double m_Mass;
		double m_Dimension; //Dimension- Length for linear and diameter for angular
		bool m_IsAngular;
};

class Entity2D;
class COMMON_API Entity_Properties
{
	public:
		Entity_Properties();
		virtual ~Entity_Properties() {}
		//The prep script takes care of the outer layer global table setup
		//override to search the appropriate global table
		virtual const char *SetUpGlobalTable(Scripting::Script& script) {return script.GetGlobalTable(m_EntityName.c_str());}
		virtual void LoadFromScript(Scripting::Script& script);
		void Initialize(Entity2D *NewEntity) const;
	protected:
		std::string m_EntityName;  //derived classes can let base class know what type to read
	private:
		//std::string m_NAME;  <-do not need this
		//Stuff needed for physics
		double m_Mass;
		double m_Dimensions[2]; //Dimensions- Length Width

		//Note: this is in Transmitted entity, but I don't think it belongs here as it doesn't describe what, but rather where
		//! Positions in meters, rotation in degrees
		//double m_X, m_Y, m_Heading;
};
