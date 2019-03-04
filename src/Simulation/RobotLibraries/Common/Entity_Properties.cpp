#include "../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../Base/Vec2d.h"
#include "../Base/Misc.h"
#include "../Base/Event.h"
#include "../Base/EventMap.h"
#include "../Base/Script.h"
#include "Entity_Properties.h"
#include "Physics_1D.h"
#include "Physics_2D.h"
#include "Entity2D.h"
#include "Goal.h"
#include "Ship_1D.h"
#include "Ship.h"

using namespace Framework::Base;

  /***********************************************************************************************************************************/
 /*															Entity1D_Properties														*/
/***********************************************************************************************************************************/

Entity1D_Properties::Entity1D_Properties()
{
	m_EntityName="Entity1D";
	m_Mass=10000.0;
	m_Dimension=12.0;
	m_IsAngular=false;
	m_StartingPosition=0.0;
}

Entity1D_Properties::Entity1D_Properties(const char EntityName[],double Mass,double Dimension,bool IsAngular)
{
	m_EntityName=EntityName;
	m_Mass=Mass;
	m_Dimension=Dimension;
	m_IsAngular=IsAngular;
	m_StartingPosition=0.0;
}

void Entity1D_Properties::LoadFromScript(Scripting::Script& script, bool NoDefaults)
{
	const char* err=NULL;

	//err = script.GetGlobalTable(m_EntityName.c_str());
	//ASSERT_MSG(!err, err);
	{
		script.GetField("mass_kg", NULL, NULL, &m_Mass);
		//At this level I do not know if I am dealing with a ship or robot, so I offer all units of measurement
		err=script.GetField("length_m", NULL, NULL,&m_Dimension);
		if (err)
		{
			double dimension;
			err=script.GetField("length_in", NULL, NULL,&dimension);
			if (!err)
				m_Dimension=Inches2Meters(dimension);
			else
			{
				err=script.GetField("length_ft", NULL, NULL,&dimension);
				if (!err)
					m_Dimension=Feet2Meters(dimension);
			}
		}
		err=script.GetField("starting_position", NULL, NULL,&m_StartingPosition);
		if (err)
		{
			double fTest;
			err=script.GetField("starting_position_deg", NULL, NULL,&fTest);
			if (!err)
				m_StartingPosition=DEG_2_RAD(fTest);
		}
	}
}

void Entity1D_Properties::Initialize(Entity1D *NewEntity) const
{
	NewEntity->m_Dimension=m_Dimension;
	NewEntity->GetPhysics().SetMass(m_Mass);
	NewEntity->m_IsAngular=m_IsAngular;
	NewEntity->m_StartingPosition=m_StartingPosition;
}

  /***********************************************************************************************************************************/
 /*														Ship_1D_Properties															*/
/***********************************************************************************************************************************/

Ship_1D_Properties::Ship_1D_Properties()
{
	double Scale=0.2;  //we must scale everything down to see on the view
	m_Ship_1D_Props.MAX_SPEED = m_Ship_1D_Props.MaxSpeed_Forward = 400.0 * Scale;
	m_Ship_1D_Props.MaxAccelReverse = -m_Ship_1D_Props.MAX_SPEED;
	m_Ship_1D_Props.ACCEL = 60.0 * Scale;
	m_Ship_1D_Props.BRAKE = 50.0 * Scale;

	m_Ship_1D_Props.MaxAccelForward=87.0 * Scale;
	m_Ship_1D_Props.MaxAccelReverse=70.0 * Scale;
	m_Ship_1D_Props.MinRange=m_Ship_1D_Props.MaxRange=0.0;
	m_Ship_1D_Props.UsingRange=false;
	m_Ship_1D_Props.DistanceDegradeScalar=1.0;  //only can be changed in script!
}

Ship_1D_Properties::Ship_1D_Properties(const char EntityName[], double Mass,double Dimension,
				   double MAX_SPEED,
				   double ACCEL,double BRAKE,
				   double MaxAccelForward, double MaxAccelReverse,
				   Ship_Type ShipType,bool UsingRange,
				   double MinRange, double MaxRange,bool IsAngular
				   ) : Entity1D_Properties(EntityName,Mass,Dimension,IsAngular)
{
	m_Ship_1D_Props.MAX_SPEED = m_Ship_1D_Props.MaxSpeed_Forward = MAX_SPEED;
	m_Ship_1D_Props.MaxSpeed_Reverse = -MAX_SPEED;
	m_Ship_1D_Props.ACCEL = ACCEL;
	m_Ship_1D_Props.BRAKE = BRAKE;
	m_Ship_1D_Props.MaxAccelForward=MaxAccelForward;
	m_Ship_1D_Props.MaxAccelReverse=MaxAccelReverse;
	m_Ship_1D_Props.ShipType=ShipType;
	m_Ship_1D_Props.MinRange=MinRange;
	m_Ship_1D_Props.MaxRange=MaxRange;
	m_Ship_1D_Props.UsingRange=UsingRange;
	m_Ship_1D_Props.DistanceDegradeScalar=1.0;  //only can be changed in script!
}

void Ship_1D_Properties::LoadFromScript(Scripting::Script& script, bool NoDefaults)
{
	const char* err=NULL;
	m_Ship_1D_Props.ShipType=Ship_1D_Props::eDefault;

	{
		double fValue;
		SCRIPT_INIT_DOUBLE(m_Ship_1D_Props.MAX_SPEED,"max_speed");
		m_Ship_1D_Props.MaxSpeed_Forward=m_Ship_1D_Props.MAX_SPEED;
		m_Ship_1D_Props.MaxSpeed_Reverse=-m_Ship_1D_Props.MAX_SPEED;

		SCRIPT_INIT_DOUBLE(m_Ship_1D_Props.MaxSpeed_Forward,"max_speed_forward");
		SCRIPT_INIT_DOUBLE(m_Ship_1D_Props.MaxSpeed_Reverse,"max_speed_reverse");

		SCRIPT_INIT_DOUBLE(	m_Ship_1D_Props.ACCEL,			"accel");
		SCRIPT_INIT_DOUBLE(	m_Ship_1D_Props.BRAKE,			"brake");
		SCRIPT_INIT_DOUBLE(	m_Ship_1D_Props.MaxAccelForward,"max_accel_forward");
		SCRIPT_INIT_DOUBLE(	m_Ship_1D_Props.MaxAccelReverse,"max_accel_reverse");
		double range;
		err=script.GetField("min_range_deg", NULL, NULL, &range);
		if (!err) m_Ship_1D_Props.MinRange=DEG_2_RAD(range);
		else
		{
			err=script.GetField("min_range", NULL, NULL, &range);
			if (!err) m_Ship_1D_Props.MinRange=range;
		}
		err=script.GetField("max_range_deg", NULL, NULL, &range);
		if (!err) m_Ship_1D_Props.MaxRange=DEG_2_RAD(range);
		else
		{
			err=script.GetField("max_range", NULL, NULL, &range);
			if (!err) m_Ship_1D_Props.MaxRange=range;
		}
		err=script.GetField("distance_scale", NULL, NULL, &range);
		if (!err) m_Ship_1D_Props.DistanceDegradeScalar=range;

		std::string sTest;
		//TODO determine why the bool type fails
		//script.GetField("using_range", NULL, &m_Ship_1D_Props.UsingRange, NULL);
		err=script.GetField("using_range", &sTest, NULL, NULL);
		if (!err)
			m_Ship_1D_Props.UsingRange=!((sTest.c_str()[0]=='n')||(sTest.c_str()[0]=='N')||(sTest.c_str()[0]=='0'));

	}
	// Let the base class finish things up
	__super::LoadFromScript(script,NoDefaults);
}

void Ship_1D_Props::SetFromShip_Properties (const Ship_Props & NewValue)
{
	MAX_SPEED=NewValue.MAX_SPEED;
	MaxSpeed_Forward=MAX_SPEED;
	MaxSpeed_Reverse=-MAX_SPEED;
	ACCEL=NewValue.ACCEL;
	BRAKE=NewValue.BRAKE;
	MaxAccelForward=NewValue.MaxAccelForward_High;
	MaxAccelReverse=NewValue.MaxAccelReverse_High;
}

void Ship_1D_Properties::UpdateShip1DProperties(const Ship_1D_Props &props)
{
	m_Ship_1D_Props=props;
}

  /***********************************************************************************************************************************/
 /*															Entity_Properties														*/
/***********************************************************************************************************************************/

Entity_Properties::Entity_Properties()
{
	m_EntityName="Entity";
	//m_NAME="default";
	m_Mass=25.0;
	m_Dimensions[0]=0.6477;
	m_Dimensions[1]=0.9525;
}

void Entity_Properties::LoadFromScript(Scripting::Script& script)
{
	const char* err=NULL;
	{
		err = script.GetField("Mass", NULL, NULL, &m_Mass);

		//Get the ship dimensions
		err = script.GetFieldTable("Dimensions");
		if (!err)
		{
			//If someone is going through the trouble of providing the dimension field I should expect them to provide all the fields!
			err = script.GetField("Length", NULL, NULL,&m_Dimensions[1]);
			err = script.GetField("Width", NULL, NULL,&m_Dimensions[0]);
			script.Pop();
		}
	}
}

void Entity_Properties::Initialize(Entity2D *NewEntity) const
{
	NewEntity->m_Dimensions[0]=m_Dimensions[0];
	NewEntity->m_Dimensions[1]=m_Dimensions[1];
	NewEntity->GetPhysics().SetMass(m_Mass);
}

