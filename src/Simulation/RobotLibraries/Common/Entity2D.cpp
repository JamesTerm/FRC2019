#include "../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../Base/Vec2d.h"
#include "../Base/Misc.h"
#include "../Base/Event.h"
#include "../Base/Script.h"
#include "Entity_Properties.h"
#include "../Base/EventMap.h"
#include "Physics_1D.h"
#include "Physics_2D.h"
#include "Entity2D.h"


  /***********************************************************************************************************************************/
 /*																	Entity1D														*/
/***********************************************************************************************************************************/

Entity1D::Entity1D(const char EntityName[]) : m_StartingPosition(0.0),m_Dimension(1.0),m_Position(0.0),m_Name(EntityName),m_BypassPos_Update(false)
{
	ResetPos();
}

Entity1D::~Entity1D() 
{
}

void Entity1D::Initialize(Base::EventMap& em, const Entity1D_Properties *props)
{
	m_eventMap = &em;
	if (props)
		props->Initialize(this);
}

void Entity1D::ResetPosition(double Position)
{
	//CancelAllControls();
	m_Physics.ResetVectors();
	if (!m_BypassPos_Update)
		m_Position=Position;
}

void Entity1D::ResetPos()
{
	ResetPosition(m_StartingPosition);
}

void Entity1D::TimeChange(double dTime_s)
{
	double PositionDisplacement=0.0;
	//Either we apply displacement computations here or the derived class will handle it
	if (!InjectDisplacement(dTime_s,PositionDisplacement))
		m_Physics.TimeChangeUpdate(dTime_s,PositionDisplacement);

	m_Position+=PositionDisplacement;
	if (m_IsAngular)
		NormalizeRotation(m_Position);
}


  /***********************************************************************************************************************************/
 /*																	Entity2D														*/
/***********************************************************************************************************************************/

Entity2D::Entity2D(const char EntityName[]) : 
	m_PosAtt_Read(&m_PosAtt_Buffers[0]),m_PosAtt_Write(&m_PosAtt_Buffers[1]),m_DefaultPos(Vec2D(0,0)),m_DefaultAtt(0.0),m_eventMap(NULL),
	m_Dimensions(1.0,1.0),m_Name(EntityName),m_BypassPosAtt_Update(false),m_Physics(m_att_r)
{
	ResetPos();
}

Entity2D::~Entity2D() 
{
}

void Entity2D::Initialize(Entity2D_Kind::EventMap& em, const Entity_Properties *props)
{
	m_eventMap = &em;
	if (props)
		props->Initialize(this);
}

//Note: If for some reason there are multiple threads which need to write we would need to put a critical section around this
//Ideally that will never be the case as we keep the logic thread a single piece... if we find that we did have to distribute the
//entities and put a critical section here for external forces, the critical section would succeed 99% of the time (all the time the ship
//is not having them applied)... this would then have no penalty
void Entity2D::UpdatePosAtt()
{
	//Note 2: In theory I do not believe I need to use the OS calls for this, and one thing we can do is try this vs. just a pure pointer
	//exchange.  I believe the results would be the same for both reliability and performance.  We could test both cases to make sure in the real
	//world tests.  (I am not sure if I can put enough stress here to know for sure)

	PosAtt *Temp=m_PosAtt_Write;
	//exchange the pointers this works since this is the only thread doing all of the writing
	m_PosAtt_Write=m_PosAtt_Read;
	m_PosAtt_Read=Temp;
}

void Entity2D::ResetPos()
{
	//CancelAllControls();
	m_Physics.ResetVectors();
	if (!m_BypassPosAtt_Update)
	{
		PosAtt *writePtr=m_PosAtt_Write;
		//SetPosAtt(m_origPos, FromLW_Rot(m_origAtt[0], m_origAtt[1], m_origAtt[2]));
		writePtr->m_pos_m=m_DefaultPos;
		writePtr->m_att_r=m_DefaultAtt;  //a.k.a heading
		m_att_r=m_DefaultAtt;
		//GetEventMap()->Event_Map["ResetPos"].Fire();
		UpdatePosAtt();
	}
}

void Entity2D::TimeChange(double dTime_s)
{
	Vec2D PositionDisplacement=Vec2D(0.0,0.0);
	double RotationDisplacement=0.0;
	//Either we apply displacement computations here or the derived class will handle it
	if (!InjectDisplacement(dTime_s,PositionDisplacement,RotationDisplacement))
		m_Physics.TimeChangeUpdate(dTime_s,PositionDisplacement,RotationDisplacement);
	//This is using an atomic double buffering mechanism... should be thread safe
	PosAtt *writePtr=m_PosAtt_Write;
	PosAtt *readPtr=m_PosAtt_Read;
	writePtr->m_pos_m=readPtr->m_pos_m+PositionDisplacement;

	double Rotation=readPtr->m_att_r+RotationDisplacement;
	NormalizeRotation(Rotation);
	writePtr->m_att_r=Rotation;
	m_att_r=Rotation;
	UpdatePosAtt();
}

