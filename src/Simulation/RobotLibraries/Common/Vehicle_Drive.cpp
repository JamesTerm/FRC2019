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
#include "Ship.h"
#include "AI_Base_Controller.h"
#include "Vehicle_Drive.h"

using namespace Framework::Base;
using namespace std;


  /***********************************************************************************************************************************/
 /*													Vehicle_Drive_Common															*/
/***********************************************************************************************************************************/

//Note: these member variables are arguably written to before read; however, I'm initializing to not mistake first iteration as corrupted data
Vehicle_Drive_Common::Vehicle_Drive_Common(Vehicle_Drive_Common_Interface *VehicleProps) : m_VehicleProps(VehicleProps),
	m_CachedLocalForce(Vec2D()),m_CachedTorque(0.0)
{
}

void Vehicle_Drive_Common::Vehicle_Drive_Common_ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2d &LocalForce,double Torque,double TorqueRestraint,double dTime_s)
{
	//Client code will call this explicitly from within Apply Thrusters
	//UpdateVelocities(PhysicsToUse,LocalForce,Torque,TorqueRestraint,dTime_s);

	//These are set in InterpolateThrusterChanges
	//m_CachedLocalForce=LocalForce;
	//m_CachedTorque=Torque;

	//Just cache these here to not pollute the control
	m_CachedLinearVelocity=PhysicsToUse.GetLinearVelocity();
	m_CachedAngularVelocity=PhysicsToUse.GetAngularVelocity();

	//Client code will call this explicitly from within Apply Thrusters... using accessors to the cached data
	//InterpolateThrusterChanges(m_CachedLocalForce,m_CachedTorque,dTime_s);
	//No torque restraint... restraints are applied during the update of velocities
}

bool Vehicle_Drive_Common::Vehicle_Drive_Common_InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement)
{
	const bool _InjectedDisplacement=true;
	if (_InjectedDisplacement)
	{
		PhysicsEntity_2D &PhysicsToUse=m_VehicleProps->Vehicle_Drive_GetPhysics_RW();
		const double Att_r=m_VehicleProps->Vehicle_Drive_GetAtt_r();
		Vec2d computedLinearVelocity=PhysicsToUse.GetLinearVelocity();
		double computedAngularVelocity=PhysicsToUse.GetAngularVelocity();
		PhysicsToUse.SetLinearVelocity(m_CachedLinearVelocity);
		PhysicsToUse.SetAngularVelocity(m_CachedAngularVelocity);
		PhysicsToUse.ApplyFractionalForce(LocalToGlobal(Att_r,m_CachedLocalForce),DeltaTime_s);
		PhysicsToUse.ApplyFractionalTorque(m_CachedTorque,DeltaTime_s);
		PhysicsToUse.TimeChangeUpdate(DeltaTime_s,PositionDisplacement,RotationDisplacement);

		//We must set this back so that the PID can compute the entire error
		PhysicsToUse.SetLinearVelocity(computedLinearVelocity);
		PhysicsToUse.SetAngularVelocity(computedAngularVelocity);
	}
	return _InjectedDisplacement;
}

void Vehicle_Drive_Common::InterpolateThrusterChanges(Vec2D &LocalForce,double &Torque,double dTime_s)
{
	m_CachedLocalForce=LocalForce;
	m_CachedTorque=Torque;
}

  /***********************************************************************************************************************************/
 /*															Tank_Drive																*/
/***********************************************************************************************************************************/

Tank_Drive::Tank_Drive(Vehicle_Drive_Common_Interface *Parent) : Vehicle_Drive_Common(Parent),m_pParent(Parent),
	m_LeftLinearVelocity(0.0),m_RightLinearVelocity(0.0)
{
}

void Tank_Drive::ResetPos()
{
	m_LeftLinearVelocity=m_RightLinearVelocity=0.0;
	//No longer aggregating, but could callback to Vehicle_Drive_Common if needed (currently not needed)
	//__super::ResetPos();
}

void Tank_Drive::UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2d &LocalForce,double Torque,double TorqueRestraint,double dTime_s)
{
	double TorqueRestrained=PhysicsToUse.ComputeRestrainedTorque(Torque,TorqueRestraint,dTime_s);

	const double D=m_pParent->GetWheelTurningDiameter();

	Vec2d CurrentVelocity=GlobalToLocal(m_pParent->Vehicle_Drive_GetAtt_r(),PhysicsToUse.GetLinearVelocity());
	const double Mass=m_pParent->Vehicle_Drive_GetPhysics().GetMass();
	const double FWD=((LocalForce[1]/Mass)*dTime_s)+CurrentVelocity[1];
	//FWD=IsZero(FWD)?0.0:FWD;
	const Vec2D &WheelDimensions=m_pParent->GetWheelDimensions();
	//L is the vehicle�s wheelbase
	const double L=WheelDimensions[1];
	//W is the vehicle�s track width
	const double W=WheelDimensions[0];
	const double inv_skid=1.0/cos(atan2(L,W));
	double RCW=(TorqueRestrained/Mass)*dTime_s+PhysicsToUse.GetAngularVelocity();
	//RCW=fabs(RCW)<0.3?0.0:RCW;
	double RPS=RCW / Pi2;
	RCW=RPS * (Pi * D) * inv_skid;  //D is the turning diameter

	m_LeftLinearVelocity = FWD + RCW;
	m_RightLinearVelocity = FWD - RCW;

	#if 0
	DOUT2("%f %f",FWD,RCW);
	DOUT4("%f %f ",m_LeftLinearVelocity,m_RightLinearVelocity);
	#endif
	//DOUT4("%f %f",FWD,RCW);  //Test accuracy
}

void Tank_Drive::InterpolateVelocities(double LeftLinearVelocity,double RightLinearVelocity,Vec2d &LocalVelocity,double &AngularVelocity,double dTime_s)
{
	const double D=m_pParent->GetWheelTurningDiameter();

	//const double FWD = (LeftLinearVelocity*cos(1.0)+RightLinearVelocity*cos(1.0))/2.0;
	const double FWD = (LeftLinearVelocity + RightLinearVelocity) * 0.5;
	//const double STR = (LeftLinearVelocity*sin(0.0)+ RightLinearVelocity*sin(0.0))/2.0;
	const double STR = 0.0;

	//const double HP=Pi/2;
	//const double HalfDimLength=GetWheelDimensions().length()/2;

	//Here we go it is finally working I just needed to take out the last division
	const Vec2D &WheelDimensions=m_pParent->GetWheelDimensions();
	//L is the vehicle�s wheelbase
	const double L=WheelDimensions[1];
	//W is the vehicle�s track width
	const double W=WheelDimensions[0];
	const double skid=cos(atan2(L,W));
	const double omega = ((LeftLinearVelocity*skid) + (RightLinearVelocity*-skid)) * 0.5;

	LocalVelocity[0]=STR;
	LocalVelocity[1]=FWD;

	AngularVelocity=(omega / (Pi * D)) * Pi2;

	//This is a sanity check of what the max heading can be
	//SmartDashboard::PutNumber("drive AngularVelocity",AngularVelocity);

	//This is a safety to avoid instability
	#if 0
	AngularVelocity=IsZero(omega)?0.0:omega;
	if (AngularVelocity>20.0)
		AngularVelocity=20.0;
	else if (AngularVelocity<-20.0)
		AngularVelocity=-20.0;
	#endif

	#if 0
	DOUT2("%f %f",FWD,omega);
	DOUT4("%f %f ",m_LeftLinearVelocity,m_RightLinearVelocity);
	#endif
	//DOUT5("%f %f",FWD,omega);
}

void Tank_Drive::InterpolateThrusterChanges(Vec2d &LocalForce,double &Torque,double dTime_s)
{
	const PhysicsEntity_2D &Physics=m_pParent->Vehicle_Drive_GetPhysics();
	const double Mass=Physics.GetMass();
	Vec2d OldLocalVelocity=GlobalToLocal(m_pParent->Vehicle_Drive_GetAtt_r(),Physics.GetLinearVelocity());
	Vec2d LocalVelocity;
	double AngularVelocity;
	InterpolateVelocities(m_LeftLinearVelocity,m_RightLinearVelocity,LocalVelocity,AngularVelocity,dTime_s);

	Vec2d LinearAcceleration=LocalVelocity-OldLocalVelocity;
	LocalForce=(LinearAcceleration * Mass) / dTime_s;

	//Now then we'll compute the torque
	double AngularAcceleration=AngularVelocity - Physics.GetAngularVelocity();
	Torque = (AngularAcceleration * Mass) / dTime_s;
	//call super for it to cache internally
	__super::InterpolateThrusterChanges(LocalForce,Torque,dTime_s);
}

void Tank_Drive::ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s)
{
	Vehicle_Drive_Common_ApplyThrusters(PhysicsToUse,LocalForce,LocalTorque,TorqueRestraint,dTime_s);	
	//This is now aggregated so no need to call super
	//__super::ApplyThrusters(PhysicsToUse,LocalForce,LocalTorque,TorqueRestraint,dTime_s);
}

bool Tank_Drive::InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement)
{
	//For test purposes
	//return false;
	return Vehicle_Drive_Common_InjectDisplacement(DeltaTime_s,PositionDisplacement,RotationDisplacement);
}


  /***********************************************************************************************************************************/
 /*														Swerve_Drive																*/
/***********************************************************************************************************************************/

Swerve_Drive::Swerve_Drive(Swerve_Drive_Interface *Parent) :  Vehicle_Drive_Common(Parent),m_pParent(Parent)
{
	memset(&m_Velocities,0,sizeof(SwerveVelocities));
}

void Swerve_Drive::ResetPos()
{
	memset(&m_Velocities,0,sizeof(SwerveVelocities));
	//No longer aggregating, but could callback to Vehicle_Drive_Common if needed (currently not needed)
	//__super::ResetPos();
}

void Swerve_Drive::UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2d &LocalForce,double Torque,double TorqueRestraint,double dTime_s)
{
	const double Mass=m_pParent->Vehicle_Drive_GetPhysics().GetMass();
	double TorqueRestrained=PhysicsToUse.ComputeRestrainedTorque(Torque,TorqueRestraint,dTime_s);
	const Vec2D &WheelDimensions=m_pParent->GetWheelDimensions();

	//L is the vehicle�s wheelbase
	const double L=WheelDimensions[1];
	//W is the vehicle�s track width
	const double W=WheelDimensions[0];

	//const double R = sqrt((L*L)+(W*W));
	const double R = m_pParent->GetWheelTurningDiameter();

	//Allow around 2-3 degrees of freedom for rotation.  While manual control worked fine without it, it is needed for
	//targeting goals (e.g. follow ship)

	Vec2d CurrentVelocity=GlobalToLocal(m_pParent->Vehicle_Drive_GetAtt_r(),PhysicsToUse.GetLinearVelocity());
	const double STR=((LocalForce[0]/Mass)*dTime_s)+CurrentVelocity[0];
	//STR=IsZero(STR)?0.0:STR;
	const double FWD=((LocalForce[1]/Mass)*dTime_s)+CurrentVelocity[1];
	//FWD=IsZero(FWD)?0.0:FWD;
	double RCW=(TorqueRestrained/Mass)*dTime_s+PhysicsToUse.GetAngularVelocity();
	//RCW=fabs(RCW)<0.3?0.0:RCW;
	double RPS=RCW / Pi2;
	RCW=RPS * (Pi * R);  //R is really diameter

	const double A = STR - RCW*(L/R);
	const double B = STR + RCW*(L/R);
	const double C = FWD - RCW*(W/R);
	const double D = FWD + RCW*(W/R);
	SwerveVelocities::uVelocity::Explicit &_=m_Velocities.Velocity.Named;

	_.sFL = sqrt((B*B)+(D*D)); _.aFL = atan2(B,D);
	_.sFR = sqrt((B*B)+(C*C)); _.aFR = atan2(B,C);
	_.sRL = sqrt((A*A)+(D*D)); _.aRL = atan2(A,D);
	_.sRR = sqrt((A*A)+(C*C)); _.aRR = atan2(A,C);

	//the angle velocities can be sensitive so if the RCW is zero then we should have a zero tolerance test
	if (RCW==0.0)
	{
		_.aFL=IsZero(_.aFL)?0.0:_.aFL;
		_.aFR=IsZero(_.aFR)?0.0:_.aFR;
		_.aRL=IsZero(_.aRL)?0.0:_.aRL;
		_.aRR=IsZero(_.aRR)?0.0:_.aRR;
	}

	#if 0
	DOUT2("%f %f %f",FWD,STR,RCW);
	DOUT4("%f %f %f %f",_.sFL,_.sFR,_.sRL,_.sRR);
	DOUT5("%f %f %f %f",_.aFL,_.aFR,_.aRL,_.aRR);
	#endif
	//DOUT4("%f %f %f",FWD,STR,RCW);  //Test accuracy

}

void Swerve_Drive::InterpolateVelocities(const SwerveVelocities &Velocities,Vec2d &LocalVelocity,double &AngularVelocity,double dTime_s)
{
	const SwerveVelocities::uVelocity::Explicit &_=Velocities.Velocity.Named;
	const Vec2D &WheelDimensions=m_pParent->GetWheelDimensions();
	//L is the vehicle�s wheelbase
	const double L=WheelDimensions[1];
	//W is the vehicle�s track width
	const double W=WheelDimensions[0];
	const double D = m_pParent->GetWheelTurningDiameter();

	const double FWD = (_.sFR*cos(_.aFR)+_.sFL*cos(_.aFL)+_.sRL*cos(_.aRL)+_.sRR*cos(_.aRR))*0.25;

	const double STR = (_.sFR*sin(_.aFR)+_.sFL*sin(_.aFL)+_.sRL*sin(_.aRL)+_.sRR*sin(_.aRR))*0.25;
	const double HP=Pi/2;
	//const double HalfDimLength=GetWheelDimensions().length()/2;

	//Here we go it is finally working I just needed to take out the last division
	const double omega = ((_.sFR*cos(atan2(W,L)+(HP-_.aFR))+_.sFL*cos(atan2(-W,L)+(HP-_.aFL))
		+_.sRL*cos(atan2(-W,-L)+(HP-_.aRL))+_.sRR*cos(atan2(W,-L)+(HP-_.aRR)))*0.25);

	//const double omega = (((_.sFR*cos(atan2(W,L)+(HP-_.aFR))/4)+(_.sFL*cos(atan2(-W,L)+(HP-_.aFL))/4)
	//	+(_.sRL*cos(atan2(-W,-L)+(HP-_.aRL))/4)+(_.sRR*cos(atan2(W,-L)+(HP-_.aRR))/4)));

	LocalVelocity[0]=STR;
	LocalVelocity[1]=FWD;

	AngularVelocity=(omega / (Pi * D)) * Pi2;
	//This is a safety to avoid instability
	#if 0
	AngularVelocity=IsZero(omega)?0.0:omega;
	if (AngularVelocity>20.0)
		AngularVelocity=20.0;
	else if (AngularVelocity<-20.0)
		AngularVelocity=-20.0;
	#endif

	#if 0
	DOUT2("%f %f %f",FWD,STR,AngularVelocity);
	DOUT4("%f %f %f %f",_.sFL,_.sFR,_.sRL,_.sRR);
	DOUT5("%f %f %f %f",_.aFL,_.aFR,_.aRL,_.aRR);
	#endif
	//DOUT5("%f %f %f",FWD,STR,omega);  //Test accuracy
}

void Swerve_Drive::InterpolateThrusterChanges(Vec2d &LocalForce,double &Torque,double dTime_s)
{
	const PhysicsEntity_2D &Physics=m_pParent->Vehicle_Drive_GetPhysics();
	const double Mass=Physics.GetMass();
	Vec2d OldLocalVelocity=GlobalToLocal(m_pParent->Vehicle_Drive_GetAtt_r(),Physics.GetLinearVelocity());
	Vec2d LocalVelocity;
	double AngularVelocity;
	InterpolateVelocities(m_pParent->GetSwerveVelocities(),LocalVelocity,AngularVelocity,dTime_s);

	Vec2d LinearAcceleration=LocalVelocity-OldLocalVelocity;
	LocalForce=(LinearAcceleration * Mass) / dTime_s;

	//Now then we'll compute the torque
	double AngularAcceleration=AngularVelocity - Physics.GetAngularVelocity();
	Torque = (AngularAcceleration * Mass) / dTime_s;
	//call super for it to cache internally
	__super::InterpolateThrusterChanges(LocalForce,Torque,dTime_s);
}

void Swerve_Drive::ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s)
{
	Vehicle_Drive_Common_ApplyThrusters(PhysicsToUse,LocalForce,LocalTorque,TorqueRestraint,dTime_s);
	//This is now aggregated so no need to call super
	//__super::ApplyThrusters(PhysicsToUse,LocalForce,LocalTorque,TorqueRestraint,dTime_s);
}

bool Swerve_Drive::InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement)
{
	return Vehicle_Drive_Common_InjectDisplacement(DeltaTime_s,PositionDisplacement,RotationDisplacement);
}

double Swerve_Drive::GetIntendedVelocitiesFromIndex(size_t index) const
{
	return m_Velocities.Velocity.AsArray[index];
}

double Swerve_Drive::GetSwerveVelocitiesFromIndex(size_t index) const
{
	return m_pParent->GetSwerveVelocities().Velocity.AsArray[index];
}


  /***********************************************************************************************************************************/
 /*															Butterfly_Drive															*/
/***********************************************************************************************************************************/

Butterfly_Drive::Butterfly_Drive(Swerve_Drive_Interface *Parent) : Swerve_Drive(Parent),m_GlobalStrafeVelocity(Vec2d(0.0,0.0)), 
	m_LocalVelocity(Vec2d(0.0,0.0)),m_PreviousGlobalVelocity(Vec2d(0.0,0.0))
{
	//SwerveVelocities::uVelocity::Explicit &_=m_Velocities.Velocity.Named;
	memset(&m_Velocities,0,sizeof(SwerveVelocities));
}

void Butterfly_Drive::UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2d &LocalForce,double Torque,double TorqueRestraint,double dTime_s)
{
	const double Mass=PhysicsToUse.GetMass();
	const double Heading=m_pParent->Vehicle_Drive_GetAtt_r();
	const double TorqueRestrained=PhysicsToUse.ComputeRestrainedTorque(Torque,TorqueRestraint,dTime_s);
	const Vec2D &WheelDimensions=m_pParent->GetWheelDimensions();

	//L is the vehicle�s wheelbase
	const double L=WheelDimensions[1];
	//W is the vehicle�s track width
	const double W=WheelDimensions[0];

	//const double D = sqrt((L*L)+(W*W));
	const double R = m_pParent->GetWheelTurningDiameter();

	//Allow around 2-3 degrees of freedom for rotation.  While manual control worked fine without it, it is needed for
	//targeting goals (e.g. follow ship)

	Vec2d CurrentVelocity=GlobalToLocal(m_pParent->Vehicle_Drive_GetAtt_r(),PhysicsToUse.GetLinearVelocity());
	Vec2d CentripetalAcceleration=GlobalToLocal(Heading,PhysicsToUse.GetCentripetalAcceleration_2D(dTime_s));

	//STR=IsZero(STR)?0.0:STR;
	const double FWD=((LocalForce[1]/Mass)*dTime_s)+CurrentVelocity[1]-CentripetalAcceleration[1];
	//FWD=IsZero(FWD)?0.0:FWD;
	const double inv_skid=1.0/cos(atan2(L,W));
	double RCW=(TorqueRestrained/Mass)*dTime_s+PhysicsToUse.GetAngularVelocity();
	//RCW=fabs(RCW)<0.3?0.0:RCW;
	double RPS=RCW / Pi2;
	RCW=RPS * (Pi * R) * inv_skid;  //R is really diameter

	const double C = FWD - RCW;
	const double D = FWD + RCW;
	SwerveVelocities::uVelocity::Explicit &_=m_Velocities.Velocity.Named;

	_.sFL = D;
	_.sFR = C;
	_.sRL = D;
	_.sRR = C;

	#if 0
	DOUT2("%f %f %f",FWD,STR,RCW);
	DOUT4("%f %f %f %f",_.sFL,_.sFR,_.sRL,_.sRR);
	DOUT5("%f %f %f %f",_.aFL,_.aFR,_.aRL,_.aRR);
	#endif
	//DOUT4("%f %f %f",FWD,STR,RCW);  //Test accuracy

}

double GetFrictionalForce(double Mass,double CoF,double Velocity,double DeltaTime_s,double BrakeResistence=0.0)
{
	const double gravity=9.80665;
	if (!DeltaTime_s) return 0.0;  //since we divide by time avoid division by zero
	double NormalForce=CoF*Mass*gravity;
	const double StoppingForce=(fabs(Velocity) * Mass) / DeltaTime_s;
	NormalForce=min(StoppingForce,NormalForce); //friction can never be greater than the stopping force
	double FrictionForce= NormalForce;
	//If the friction force overflows beyond stopping force, apply a scale to the overflow of force 
	if (FrictionForce>StoppingForce) FrictionForce=(BrakeResistence * (FrictionForce-StoppingForce));

	//Return the fractional force in the opposite direction of the current velocity
	return (Velocity>0.0)? -FrictionForce : FrictionForce;
}

void Butterfly_Drive::ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s)
{
	const double Mass=PhysicsToUse.GetMass();
	const double Heading=m_pParent->Vehicle_Drive_GetAtt_r();
	const Vec2d CurrentGlobalVelocity=LocalToGlobal(Heading,m_LocalVelocity);
	const Vec2d LocalDeltaVelocity=GlobalToLocal(Heading,CurrentGlobalVelocity-m_PreviousGlobalVelocity);

	//double CentripetalAcceleration=PhysicsEntity_2D::GetCentripetalAcceleration(m_LocalVelocity[1],m_CachedAngularVelocity,dTime_s);
	//CentripetalAcceleration+=((GetFrictionalForce(Mass,0.20,CentripetalAcceleration,dTime_s)/Mass) * dTime_s);
	Vec2d LocalStrafeVelocity=GlobalToLocal(Heading,Vec2d(m_GlobalStrafeVelocity[0],0.0));  //bring our cached velocity to local to add it
	//Add the centripetal acceleration to the x component to our velocity 
	LocalStrafeVelocity[0]+=-LocalDeltaVelocity[0];
	LocalStrafeVelocity[1]=0;  //This gets absorbed in the current direction
	//just hard code the CoF which allows x percent of the centripetal force to escape
	LocalStrafeVelocity[0]+=((GetFrictionalForce(Mass,0.20,LocalStrafeVelocity[0],dTime_s)/Mass) * dTime_s);
	//Cache this velocity in its global direction
	m_GlobalStrafeVelocity=LocalToGlobal(Heading,LocalStrafeVelocity);
	m_PreviousGlobalVelocity=LocalToGlobal(Heading,Vec2d(LocalStrafeVelocity[0],m_LocalVelocity[1]));
	//DOUT5 ("%f x=%f y=%f",LocalStrafeVelocity[0],Meters2Feet(m_LocalVelocity[0]),Meters2Feet(m_LocalVelocity[1]));
	m_LocalVelocity[0]=LocalStrafeVelocity[0];
	Swerve_Drive::ApplyThrusters(PhysicsToUse,LocalForce,LocalTorque,TorqueRestraint,dTime_s);
}

double Butterfly_Drive::GetStrafeVelocity(const PhysicsEntity_2D &PhysicsToUse,double dTime_s) const
{
	#if 0
	return 0.0;
	#else
	return m_LocalVelocity[0]; 
	#endif
}

void Butterfly_Drive::InterpolateVelocities(const SwerveVelocities &Velocities,Vec2d &LocalVelocity,double &AngularVelocity,double dTime_s)
{
	const SwerveVelocities::uVelocity::Explicit &_=Velocities.Velocity.Named;
	const Vec2D &WheelDimensions=m_pParent->GetWheelDimensions();
	//L is the vehicle�s wheelbase
	const double L=WheelDimensions[1];
	//W is the vehicle�s track width
	const double W=WheelDimensions[0];
	const double D = m_pParent->GetWheelTurningDiameter();

	const double FWD = (_.sFR+_.sFL+_.sRL+_.sRR)*0.25;

	const double STR = GetStrafeVelocity(m_pParent->Vehicle_Drive_GetPhysics(),dTime_s);
	//const double HP=Pi/2;
	//const double HalfDimLength=GetWheelDimensions().length()/2;

	//Here we go it is finally working I just needed to take out the last division
	//const double omega = ((_.sFR*cos(atan2(W,L)+HP))+_.sFL*cos(atan2(-W,L)+(HP))
	//	+_.sRL*cos(atan2(-W,-L)+(HP)+_.sRR*cos(atan2(W,-L)+(HP)))/4);
	const double skid=cos(atan2(L,W));
	const double omega = (_.sFR*-skid+_.sFL*skid+_.sRL*skid+_.sRR*-skid)*0.25;

	LocalVelocity[0]=STR;
	LocalVelocity[1]=FWD;
	m_LocalVelocity=LocalVelocity;

	AngularVelocity=(omega / (Pi * D)) * Pi2;
}

  /***********************************************************************************************************************************/
 /*																Nona_Drive															*/
/***********************************************************************************************************************************/

Nona_Drive::Nona_Drive(Swerve_Drive_Interface *Parent) : Butterfly_Drive(Parent),m_KickerWheel(0.0)
{
	//SwerveVelocities::uVelocity::Explicit &_=m_Velocities.Velocity.Named;
	memset(&m_Velocities,0,sizeof(SwerveVelocities));
}

void Nona_Drive::UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2d &LocalForce,double Torque,double TorqueRestraint,double dTime_s)
{
	if (!m_pParent->IsTractionMode())
	{
		const double Mass=PhysicsToUse.GetMass();
		const double Heading=m_pParent->Vehicle_Drive_GetAtt_r();
		Vec2d CentripetalAcceleration=GlobalToLocal(Heading,PhysicsToUse.GetCentripetalAcceleration_2D(dTime_s));
		//DOUT5("%f, %f, %f",CentripetalAcceleration[0],CentripetalAcceleration[1], PhysicsToUse.GetCentripetalAcceleration_Magnitude(dTime_s));

		Vec2d CurrentVelocity=GlobalToLocal(Heading,PhysicsToUse.GetLinearVelocity());
		m_KickerWheel=((LocalForce[0]/Mass)*dTime_s)+CurrentVelocity[0] - CentripetalAcceleration[0];
	}
	else
		m_KickerWheel=0.0;

	//DOUT5("%f %f",CurrentVelocity[0],m_KickerWheel);
	__super::UpdateVelocities(PhysicsToUse,LocalForce,Torque,TorqueRestraint,dTime_s);
}

double Nona_Drive::GetStrafeVelocity(const PhysicsEntity_2D &PhysicsToUse,double dTime_s) const
{
	return m_KickerWheel;
}

void Nona_Drive::ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s)
{
	//bypass butterfly implementation
	Swerve_Drive::Vehicle_Drive_Common_ApplyThrusters(PhysicsToUse,LocalForce,LocalTorque,TorqueRestraint,dTime_s);
}

