#include <math.h>
#include <assert.h>
#include "../Base/Base_Includes.h"
#include "../Base/Vec2d.h"
#include "../Base/Misc.h"
#include "Physics_2D.h"

using namespace Framework;
using namespace std;
using namespace Framework::Base;


//The actual force between two objects are f=(G m1 m2)/ r^2
//For example g = ( G * (Me->5.98E+24)) / (Re->6.38E+6)^2 = 9.8 m/s^2
//G is ideal to compute forces on ships from various planets
const double G=6.673E-11;

inline Vec2D GlobalToLocal(double Heading,const Vec2D &GlobalVector);
inline Vec2D LocalToGlobal(double Heading,const Vec2D &LocalVector);

inline bool PosBNE(double val,double t)
{
	return !(fabs(val-t)<1E-3);
}


inline const Vec2d Vec2Multiply (const Vec2d &A,const Vec2d &rhs)
{
	return Vec2d(A[0]*rhs._v[0], A[1]*rhs._v[1]);
}

inline const Vec2d Vec2_abs (const Vec2d &A)
{
	return Vec2d(fabs(A[0]),fabs( A[1]));
}
inline const Vec2d Vec2_min (const Vec2d &A,const Vec2d &B)
{
	return Vec2d(min(A[0],B[0]),min(A[1],B[1]));
}


//Normalize it in the bound of Pi2
inline void NormalizeToPi2(double &A)
{
	if (A<0)
		A+=Pi2;
	else if (A>Pi2)
		A-=Pi2;
}

inline void NormalizeToPi2wDirection(double &A)
{
	if (A<-Pi2)
		A+=Pi2;
	else if (A>Pi2)
		A-=Pi2;
}


void PhysicsEntity_2D::SetMass(double mass)
{
	m_EntityMass=mass;
}
double PhysicsEntity_2D::GetMass() const
{
	return m_EntityMass;
}

void PhysicsEntity_2D::CopyFrom(const PhysicsEntity_2D& rhs)
{
	m_LinearVelocity= rhs.m_LinearVelocity;
	m_AngularVelocity= rhs.m_AngularVelocity;
}

void PhysicsEntity_2D::ResetVectors()
{
	m_LinearVelocity= Vec2d(0,0);
	m_AngularVelocity= 0.0;
}

PhysicsEntity_2D::PhysicsEntity_2D()
{
	//Plug in some good defaults
	m_EntityMass=500; //about 5000 pounds
	//m_EntityMass=200; //about 2000 pounds
	m_StaticFriction=0.8;
	m_KineticFriction=0.2;
	m_AngularInertiaCoefficient=1.0;
	m_RadiusOfConcentratedMass=1.0;
	m_SummedExternalForces=Vec2d(0,0);
	m_lastTime_s=0.0;

	ResetVectors();
}

void PhysicsEntity_2D::SetFriction(double StaticFriction,double KineticFriction)
{
	m_StaticFriction=StaticFriction;
	m_KineticFriction=KineticFriction;
}

void PhysicsEntity_2D::SetAngularInertiaCoefficient(double AngularInertiaCoefficient)
{
	m_AngularInertiaCoefficient=AngularInertiaCoefficient;
}

void PhysicsEntity_2D::SetRadiusOfConcentratedMass(double RadiusOfConcentratedMass)
{
	m_RadiusOfConcentratedMass=RadiusOfConcentratedMass;
}

double PhysicsEntity_2D::GetRadiusOfConcentratedMass() const
{
	return m_RadiusOfConcentratedMass;
}

void PhysicsEntity_2D::SetLinearVelocity( const Vec2d &LinearVelocity)
{ 
	m_LinearVelocity=LinearVelocity;
}
Vec2d PhysicsEntity_2D::GetLinearVelocity() const
{
	return m_LinearVelocity;
}

void PhysicsEntity_2D::SetAngularVelocity( double AngularVelocity)
{
	m_AngularVelocity=AngularVelocity;
}
double PhysicsEntity_2D::GetAngularVelocity() const
{
	return m_AngularVelocity;
}

void PhysicsEntity_2D::ApplyFractionalForce( const Vec2d &force,double FrameDuration)
{
	//I'm writing this out so I can easily debug
	Vec2d AccelerationDelta=force/m_EntityMass;
	Vec2d VelocityDelta=AccelerationDelta*FrameDuration;
	m_LinearVelocity+=VelocityDelta;

	//if (AccelerationDelta[1]!=0)
	//	DebugOutput("Acc%f Vel%f\n",AccelerationDelta[1],m_LinearVelocity[1]);
}
void PhysicsEntity_2D::ApplyFractionalTorque( double torque,double FrameDuration,double RadialArmDistance)
{
	double AccelerationDelta=GetAngularAccelerationDelta(torque,RadialArmDistance);
	double VelocityDelta=AccelerationDelta*FrameDuration;
	m_AngularVelocity+=VelocityDelta;
}

void PhysicsEntity_2D::ApplyFractionalForce( const Vec2d &force, const Vec2d &point,double FrameDuration )
{
	//Use this as a "get by" if the code doesn't work properly
#if 0
	ApplyFractionalForce(force,FrameDuration);
	return;
#endif
	//Here is a rough draft to solve in 2 dimensions
	//A=atan2(py,px)   point
	//M=pi/2 - A
	//L=atan2(fy,fx)  force
	//N=L+M
	//Y=sin(N)*f.length = contribution for force
	//X=cos(N)*f.length = contribution for torque

	double TorqueToApply;
	Vec2d ForceToApply;
	double RadialArmDistance;

	{
		double A=atan2(point[1],point[0]);
		double M=(M_PI/2) - A;
		double L=atan2(-force[1],-force[0]);
		double N=L+M;

		double ForceLength= sqrt((force[1]*force[1])+(force[0]*force[0]));
		RadialArmDistance= sqrt((point[1]*point[1])+(point[0]*point[0]));
		//I've reserved a special case for ships which haven't specified  their radius size, in which case we simply factor out the radial arm too
		if ((m_RadiusOfConcentratedMass==1.0)&&(RadialArmDistance>1.0)) RadialArmDistance=1.0;

		//Fr = t   ... We should multiply force by the radial arm distance to get the torque
		//but instead,  we pass it off to physics where the multiply gets applied directly against the Radius of Concentrated Mass
		//We could multiply here but doing it the other way keeps the torque value low, which also makes it easier to debug
		TorqueToApply=(cos(N)*ForceLength); 
	}

	Vec2d vecToCenter = -point;
	//Note we should be able to support a point set at 0,0,0 in which case we use the force itself as the direction... otherwise a zero'd point
	//results in a zero'd vector which would omit applying the force
	if (vecToCenter.length2()==0.0)
		vecToCenter=-force;

	vecToCenter.normalize();

	ForceToApply = vecToCenter * (force * vecToCenter);

	ApplyFractionalForce(ForceToApply,FrameDuration);
	ApplyFractionalTorque(TorqueToApply,FrameDuration,RadialArmDistance);
}

inline double PhysicsEntity_2D::GetAngularAccelerationDelta(double torque,double RadialArmDistance)
{
	/* We want a cross product here, and divide by the mass and angular inertia
	return (RadialArmDistance^torque) / (m_EntityMass*m_AngularInertiaCoefficient);

	// [Rick Notes], Why divide by the arm distance?  Shouldn't we be multiplying?  Why square that, and along just the component?
	// We divide by I to solve for a... see formula below
	*/

	// t=Ia 
	//I=sum(m*r^2) or sum(AngularCoef*m*r^2)

	double ret;
	{
		//Avoid division by zero... no radial arm distance no acceleration!
		if (RadialArmDistance==0)
		{
			ret=0;
			return ret;
		}
		//Doing it this way keeps the value of torque down to a reasonable level
		// [Rick Notes]  What does a "Reasonable Level" mean?  Perhaps we should see the equation somewhere
		// I forgot what the equation was and I get a bit lost.
		double RadiusRatio(m_RadiusOfConcentratedMass*m_RadiusOfConcentratedMass/RadialArmDistance);
		assert(RadiusRatio!=0);  //no-one should be using a zero sized radius!
		ret=(torque/(m_AngularInertiaCoefficient*m_EntityMass*RadiusRatio));
	}
	return ret;
}

Vec2d PhysicsEntity_2D::GetForceFromVelocity(const Vec2d &vDesiredVelocity,double DeltaTime_s) const
{
	Vec2d DeltaVelocity=(vDesiredVelocity-GetLinearVelocity());
	//A=Delta V / Delta T
	Vec2d Acceleration=DeltaVelocity/DeltaTime_s;  //This may be pretty quick (apply Force restrictions later)


	//if (Acceleration!=Vec2d(0,0,0))
	//	printf(" x=%f,y=%f,z=%f\n",Acceleration[0],Acceleration[1],Acceleration[2]);
	//Now that we know what the acceleration needs to be figure out how much force to get it there
	Vec2d Force= Acceleration * m_EntityMass;
	//if (PosBNE(Force[0],0)||(PosBNE(Force[1],0))||(PosBNE(Force[2],0)))
	//	printf("tx=%f,ty=%f,tz=%f\n",Force[0],Force[1],Force[2]);

	//if (PosBNE(Heading[2],0.0))
	//	DebugOutput(" s=%f,a=%f,w=%f,h=%f,z=%f,t=%f\n",Distance[2],m_AngularAcceleration[2],m_AngularVelocity[2],Heading[2],CurrentOrientation[2],Force[2]);

	return Force;
}

double PhysicsEntity_2D::GetTorqueFromVelocity( double vDesiredVelocity,double DeltaTime_s) const
{

	//TODO input torque restraints from script (this would be due to the capabilities of the engines)
	//Note: desired speed is a separated variable controlled from the ship's speed script, which we fine tune given the torque restraints
	//And also by minimizing the amount of G's pulled at the outer most edge of the ship... so for large ships that rotate this could be
	//significant, and you wouldn't want people slamming into the walls.
	//Note: if the speed is too high and the torque restraint is too low the ship will "wobble" because it trying to to go a faster speed that it
	//can "brake" for... ideally a little wobble is reasonable and this is controlled by a good balance between desired speed and torque restraints


	double DeltaVelocity=(vDesiredVelocity-GetAngularVelocity());
	//A=Delta V / Delta T
	double Acceleration=DeltaVelocity/DeltaTime_s;  //This may be pretty quick (apply torque restrictions later)

	//if (Acceleration!=Vec2d(0,0,0))
	//	printf(" x=%f,y=%f,z=%f\n",Acceleration[0],Acceleration[1],Acceleration[2]);
	//Now that we know what the acceleration needs to be figure out how much force to get it there
	double Torque= Acceleration * m_EntityMass;
	//if (PosBNE(Torque[0],0)||(PosBNE(Torque[1],0))||(PosBNE(Torque[2],0)))
	//	printf("tx=%f,ty=%f,tz=%f\n",Torque[0],Torque[1],Torque[2]);

	//if (PosBNE(Heading[2],0.0))
	//	DebugOutput(" s=%f,a=%f,w=%f,h=%f,z=%f,t=%f\n",Distance[2],m_AngularAcceleration[2],m_AngularVelocity[2],Heading[2],CurrentOrientation[2],Torque[2]);

	return Torque;
}

double PhysicsEntity_2D::GetVelocityFromDistance_Angular(double Distance,double Restraint,double DeltaTime_s,double matchVel,bool normalize)
{
	double ret;

	//This is how many radians the ship is capable to turn for this given time frame
	double Acceleration=(Restraint/m_EntityMass); //obtain acceleration

	{
		//first compute which direction to go
		double DistanceDirection=Distance;
		DistanceDirection-=matchVel*DeltaTime_s;
		if (IsZero(DistanceDirection))
		{
			ret=matchVel;
			return ret;
		}

		if (normalize)
		{
			//Unlike in the 3D physics, we'll need while loops to ensure all of the accumulated turns are normalized, in the 3D physics the
			//Quat is auto normalized to only require one if check here
			while (DistanceDirection>M_PI)
				DistanceDirection-=Pi2;
			while (DistanceDirection<-M_PI)
				DistanceDirection+=Pi2;
		}

		double DistanceLength=fabs(DistanceDirection);

		//Ideal speed needs to also be normalized
		double IDS=Distance;
		if (normalize)
		{
			if (IDS>M_PI)
				IDS-=Pi2;
			else if (IDS<-M_PI)
				IDS+=Pi2;
		}

		double IdealSpeed=fabs(IDS/DeltaTime_s);

		if (Restraint!=-1)
		{
			//Given the distance compute the time needed
			//Place the division first keeps the multiply small
			double Time=sqrt(2.0*(DistanceLength/Acceleration));
			//With torque and its fixed point nature... it is important to have the jump ahead of the slope so that it doesn't overshoot
			//this can be accomplished by subtracting this delta time and working with that value... this should work very well but it could
			//be possible for a slight overshoot when the delta times slices are irregular. 
			if (Time>DeltaTime_s)
			{
				Time-=DeltaTime_s;
				if (IsZero(Time))
					Time=0.0;
			}
			//Now compute maximum speed for this time
			double MaxSpeed=Acceleration*Time;
			ret=min(IdealSpeed,MaxSpeed);

			if (DistanceDirection<0)
				ret=-ret;
			ret+=matchVel;
		}
		else
		{
			ret=IdealSpeed;  //i.e. god speed
			if (IDS<0)
				ret=-ret;
		}
	}
	#if 0
	if (fabs(m_AngularVelocity)>0.0)
		printf("y=%.2f p=%.2f e=%.2f cs=0\n",Distance,ret,m_AngularVelocity);
	#endif

	return ret;
}

inline Vec2d PhysicsEntity_2D::GetVelocityFromCollision(Vec2d ThisVelocityToUse,double otherEntityMass,Vec2d otherEntityVelocity)
{
	//almost not quite
	//return (m_LinearVelocity*(m_EntityMass-otherEntityMass)) / (m_EntityMass+otherEntityMass);

	/// en.wikipedia.org/wiki/Elastic_collision
	// Here is the equation
	// ((vel1 ( mass1 - mass2 )) + (2 * mass2 * vel2))) / (m1+m2)
	Vec2d ret= (ThisVelocityToUse *(m_EntityMass-otherEntityMass));
	ret+= (otherEntityVelocity*(2 * otherEntityMass));
	ret/= (m_EntityMass+otherEntityMass);
	return ret;
}


//just like GetVelocityFromDistance_Angular except we do not normalize the DistanceDirection
Vec2d PhysicsEntity_2D::GetVelocityFromDistance_Linear(const Vec2d &Distance,const Vec2d &ForceRestraintPositive,const Vec2d &ForceRestraintNegative,double DeltaTime_s, const Vec2d& matchVel)
{
	Vec2d ret;
	//These are initialized as we go
	double Acceleration;
	double Restraint;

	for (size_t i=0;i<2;i++)
	{
		double DistanceDirection=Distance[i];
		DistanceDirection-=matchVel[i]*DeltaTime_s;
		if (IsZero(DistanceDirection))
		{
			ret[i]=matchVel[i];
			continue;
		}
		double DistanceLength=fabs(DistanceDirection);

		//Compose our restraint and acceleration based on the component direction
		Restraint=(DistanceDirection>0)?ForceRestraintPositive[i]:ForceRestraintNegative[i];
		Acceleration=(Restraint/m_EntityMass);

		double IdealSpeed=fabs(Distance[i]/DeltaTime_s);

		if (Restraint!=-1)
		{
			//Given the distance compute the time needed
			//Place the division first keeps the multiply small
			double Time=sqrt(2.0*(DistanceLength/Acceleration));

			if (Time>DeltaTime_s)
			{
				Time-=DeltaTime_s;
				if (IsZero(Time))
					Time=0.0;
			}

			//Now compute maximum speed for this time
			double MaxSpeed=Acceleration*Time;
			ret[i]=min(IdealSpeed,MaxSpeed);

			if (DistanceDirection<0.0)
				ret[i]=-ret[i];
			ret[i]+=matchVel[i];
		}
		else
		{
			ret[i]=IdealSpeed;  //i.e. god speed
			if (Distance[i]<0.0)
				ret[i]=-ret[i];
		}
	}
	return ret;
}

Vec2d PhysicsEntity_2D::GetVelocityFromDistance_Linear_v1(const Vec2d &Distance,const Vec2d &ForceRestraintPositive,const Vec2d &ForceRestraintNegative,double DeltaTime_s, const Vec2d& matchVel)
{
	Vec2d ret;
	Vec2d DistToUse=Distance; //save the original distance as it is const anyhow

	DistToUse-=(matchVel*DeltaTime_s);
	double dDistance=DistToUse.length();
	if (IsZero(dDistance))
		return matchVel;

	//This is how many meters per second the ship is capable to stop for this given time frame
	Vec2d Restraint;
	//Compute the restraint based off of its current direction
	for (size_t i=0;i<3;i++)
		Restraint[i]=DistToUse[i]>0?ForceRestraintPositive[i]:ForceRestraintNegative[i];

	Vec2d Acceleration=(Restraint/m_EntityMass); //obtain acceleration

	double IdealSpeed=Distance.length()/DeltaTime_s;
	double AccelerationMagnitude=Acceleration.length();
	double Time=sqrt(2.0*(dDistance/AccelerationMagnitude));

	double MaxSpeed=AccelerationMagnitude*Time;
	double SpeedToUse=min(IdealSpeed,MaxSpeed);

	//DebugOutput("d=%f i=%f m=%f\n",Distance[1],IdealSpeed,MaxSpeed);
	//Now to give this magnitude a direction based of the velocity
	double scale=SpeedToUse/dDistance;
	ret=DistToUse*scale;
	ret+=matchVel;
	return ret;
}

double PhysicsEntity_2D::ComputeRestrainedTorque(double Torque,double TorqueRestraint,double dTime_s)
{  
	double TorqueToApply=Torque;

	if (TorqueRestraint!=-1)
	{
		double SmallestRatio=1.0;
		{
			double AbsComponent=fabs(TorqueToApply);
			if ((AbsComponent>TorqueRestraint) && (TorqueRestraint > 0.0))
			{
				double Temp=TorqueRestraint/AbsComponent;
				SmallestRatio=Temp<SmallestRatio?Temp:SmallestRatio;
			}
		}
		TorqueToApply*=SmallestRatio;
	}
	
	return TorqueToApply;
}


Vec2d PhysicsEntity_2D::ComputeRestrainedForce(const Vec2d &LocalForce,const Vec2d &ForceRestraintPositive,const Vec2d &ForceRestraintNegative,double dTime_s)
{
	Vec2d ForceToApply=LocalForce;
	if (ForceRestraintPositive[0]!=-1)
	{
		double SmallestRatio=1.0;
		//Apply Force restraints; This method computes the smallest ratio needed to scale down the vector.  It should give the maximum amount
		//of magnitude available without sacrificing the intended direction
		for (size_t i=0;i<2;i++)
		{
			double Temp;
			//separate the positive and negative coordinates
			if (LocalForce[i]>0)
			{
				if (LocalForce[i]>ForceRestraintPositive[i])
				{
					Temp=ForceRestraintPositive[i]/LocalForce[i];
					SmallestRatio=Temp<SmallestRatio?Temp:SmallestRatio;
				}
			}
			else
			{
				double AbsComponent=fabs(LocalForce[i]);
				if (AbsComponent>ForceRestraintNegative[i])
				{
					Temp=ForceRestraintNegative[i]/AbsComponent;
					SmallestRatio=Temp<SmallestRatio?Temp:SmallestRatio;
				}
			}
		}
		ForceToApply*=SmallestRatio;
		//Test my restraints
		//printf("\r lr %f fr %f ud %f                ",LocalForce[0],LocalForce[1],LocalForce[2]);
	}
	return ForceToApply;
}

double PhysicsEntity_2D::GetCentripetalAcceleration(double LinearVelocity,double AngularVelocity,double DeltaTime_s)
{
	//centripetal_a = v^2 / r
	//first we'll need to find r given the current angular velocity
	//r = s / theta  (where theta is rotational displacement or angular velocity * time)
	const double theta=LinearVelocity>0.0?-AngularVelocity:AngularVelocity;
	if (IsZero(theta)) return 0.0;
	const double v = LinearVelocity;
	const double s =v / DeltaTime_s;
	const double r = s / theta;
	if (IsZero(r)) return 0.0;
	const double centripetal_acceleration= v * v / r;
	return LinearVelocity>0.0?centripetal_acceleration:-centripetal_acceleration;
}

double PhysicsEntity_2D::GetCentripetalAcceleration(double DeltaTime_s) const
{
	//Note the length of linear velocity works because it does not matter which direction the velocity is facing
	return GetCentripetalAcceleration(m_LinearVelocity.length(),m_AngularVelocity,DeltaTime_s);
}

Vec2d PhysicsEntity_2D::GetCentripetalAcceleration_2D(double DeltaTime_s) const
{
	return GetCentripetalForce(DeltaTime_s) / m_EntityMass;
}

//TODO determine why this isn't building
inline Vec2D GlobalToLocal2(double Heading,const Vec2D &GlobalVector)
{
	return Vec2D(sin(-Heading)*GlobalVector[1]+cos(Heading)*GlobalVector[0],
		cos(-Heading)*GlobalVector[1]+sin(Heading)*GlobalVector[0]);
}

Vec2d PhysicsEntity_2D::GetCentripetalForce(double DeltaTime_s) const
{
	//F_centripetal = m v^2 / r
	//return GetCentripetalAcceleration(DeltaTime_s) * m_EntityMass;

	//This way is more efficient
	Vec2d AlteredVelocity=GlobalToLocal2(m_AngularVelocity*DeltaTime_s,m_LinearVelocity);
	return GetForceFromVelocity(AlteredVelocity,DeltaTime_s) * DeltaTime_s;
}

void PhysicsEntity_2D::TimeChangeUpdate(double DeltaTime_s,Vec2d &PositionDisplacement,double &RotationDisplacement)
{
	//Transfer the velocity to displacement
	RotationDisplacement = m_AngularVelocity * DeltaTime_s;
	PositionDisplacement = m_LinearVelocity * DeltaTime_s;
}


  /************************************************************************************************************************/
 /*										                  FlightDynamics_2D							                     */
/************************************************************************************************************************/

void FlightDynamics_2D::init()
{
	//populate LinearAccelerationRates with some good defaults
	const Vec2d AccDelta(30.0,30.0);
	const Vec2d DecDelta(60.0,60.0);
	LinearAccelerationRates &_=m_LinearAccelerationRates;
	_.AccDeltaNeg=_.AccDeltaPos=AccDelta;
	_.DecDeltaNeg=_.DecDeltaPos=DecDelta;
	m_UsingAccelerationRate=false;
	StructuralDmgGLimit = 10.0;
	G_Dampener = 1.0;
}

FlightDynamics_2D::FlightDynamics_2D() : m_HeadingToUse(&m_DefaultHeading)
{
	init();
}

FlightDynamics_2D::FlightDynamics_2D(const double *HeadingToUse) : m_HeadingToUse(HeadingToUse)
{
	init();
}

FlightDynamics_2D::FlightDynamics_2D(const double &HeadingToUse) : m_HeadingToUse(&HeadingToUse)
{
	init();
}

void FlightDynamics_2D::ResetVectors()
{
	__super::ResetVectors();
	if (m_HeadingToUse==&m_DefaultHeading)
		m_DefaultHeading=0.0;
	m_CurrentAcceleration=m_TargetAcceleration=Vec2d(0.0,0.0);
}

__inline double shortest_angle(double Distance)
{
	return Distance - Pi2*floor(Distance/Pi2+0.5);
}

double FlightDynamics_2D::ComputeAngularDistance(const Vec2d &lookDir)
{
	double lookDir_radians= atan2(lookDir[0],lookDir[1]);
	double distance=*m_HeadingToUse-lookDir_radians;
	return shortest_angle(distance);
}

Vec2d FlightDynamics_2D::ComputeAngularDistance_asLookDir(const Vec2d &lookDir)
{
	double distance=ComputeAngularDistance(lookDir);
	return Vec2d(sin(distance),cos(distance));
}

#if 0
Vec2d FlightDynamics_2D::ComputeAngularDistance(double Orientation)
{
	double distance=Orientation-m_HeadingToUse;
	return Vec2d(sin(distance),cos(distance));
}
#endif

double FlightDynamics_2D::ComputeAngularDistance(double Orientation)
{
	double DistanceDirection= *m_HeadingToUse-Orientation;
	return shortest_angle(DistanceDirection);
}

void FlightDynamics_2D::TimeChangeUpdate(double DeltaTime_s,Vec2d &PositionDisplacement,double &RotationDisplacement)
{
	__super::TimeChangeUpdate(DeltaTime_s,PositionDisplacement,RotationDisplacement);
	if (m_HeadingToUse==&m_DefaultHeading)
		m_DefaultHeading+=RotationDisplacement;
}


FlightDynamics_2D::LinearAccelerationRates &FlightDynamics_2D::GetLinearAccelerationRates()
{
	return m_LinearAccelerationRates;
}

void FlightDynamics_2D::SetTargetAcceleration(const Vec2d &TargetAcceleration)
{
	m_TargetAcceleration=TargetAcceleration;
}

Vec2d FlightDynamics_2D::GetAcceleration_Delta(double dTime_s,const Vec2d &TargetAcceleration,bool Clipping)
{
	//implicitly initialized in case we need to do no work
	Vec2d ret(0.0,0.0);
	//Note: this is somewhat simplified when we cross-over the zero thresh-hold, where instead of blending one derivative against the other
	//it will follow through with first tagged one.  Since the deltas are encapsulated it should work, because it will still
	//accurately predict the force that will be applied.  The amount of error for this case should be minimal enough to not be noticeable.
	//We can provide different engine models in the future
	//Compute the acceleration given the derivatives
	LinearAccelerationRates &_=m_LinearAccelerationRates;
	for (size_t i=0;i<2;i++)
	{
		if ((TargetAcceleration[i]==m_CurrentAcceleration[i])&&(Clipping))
			continue;
		//determine if we are going in a positive or negative direction
		if (TargetAcceleration[i]>m_CurrentAcceleration[i]) //We are going in a positive direction
		{
			double Delta=m_CurrentAcceleration[i]>0?_.AccDeltaPos[i]:_.DecDeltaNeg[i];
			Delta*=dTime_s;
			if (Clipping)
				ret[i]=min(Delta,TargetAcceleration[i]-m_CurrentAcceleration[i]);
			else
				ret[i]=Delta;
		}
		else //We are going in a negative direction
		{
			double Delta=m_CurrentAcceleration[i]>0?_.DecDeltaPos[i]:_.AccDeltaNeg[i];
			Delta*=dTime_s;
			if (Clipping)
				ret[i]=max(-Delta,TargetAcceleration[i]-m_CurrentAcceleration[i]);
			else
				ret[i]=Delta;
		}
	}
	return ret;
}

Vec2d FlightDynamics_2D::GetAcceleration_Delta (double dTime_s)
{
	return GetAcceleration_Delta(dTime_s,m_TargetAcceleration);
}

void FlightDynamics_2D::Acceleration_TimeChangeUpdate(double dTime_s)
{
	const Vec2d Acceleration_Delta=GetAcceleration_Delta(dTime_s);
	m_CurrentAcceleration+=Acceleration_Delta;
}

Vec2d FlightDynamics_2D::GetForceFromVelocity(const Vec2d &vDesiredVelocity,double DeltaTime_s)
{
	Vec2d Force;
	if (!m_UsingAccelerationRate)
		Force=__super::GetForceFromVelocity(vDesiredVelocity,DeltaTime_s);
	else
	{
		Vec2d Zerod=Vec2d(0.0,0.0);
		Vec2d Acceleration=Zerod;
		Vec2d HeadingToUse_NV=Vec2d(sin(*m_HeadingToUse),cos(*m_HeadingToUse));
		const Vec2d DeltaVelocity=(vDesiredVelocity-GetLinearVelocity());
		//compute the maximum deceleration speed, since we need to reach 0 by the time we reach our desired velocity
		//Note: these are in local orientation so they need to be converted to global
		const Vec2d MaxDeceleration=Vec2Multiply(HeadingToUse_NV , GetAcceleration_Delta(DeltaTime_s,Vec2d(0.0,0.0),false));
		Vec2d Global_CurrentAcceleration(Vec2Multiply(HeadingToUse_NV , m_CurrentAcceleration));
		
		//A=Delta V / Delta T
		const Vec2d NewAcceleration_target=DeltaVelocity/DeltaTime_s;  //This may be pretty quick (apply Force restrictions later)

		{
			double MaxDeceleration_Length=MaxDeceleration.length();
			double CurrentAcceleration_Length=Global_CurrentAcceleration.length();
			double SpeedUpTime=CurrentAcceleration_Length?DeltaVelocity.length()/(CurrentAcceleration_Length*DeltaTime_s):0.0;
			double SpeedDownTime=MaxDeceleration_Length?CurrentAcceleration_Length/MaxDeceleration_Length:0.0;
			//Choose whether to speed up or slow down based on which would take longer to do
			if ((SpeedUpTime>SpeedDownTime)||(MaxDeceleration_Length>=CurrentAcceleration_Length))
				Acceleration=NewAcceleration_target;
			else
				Acceleration=DeltaVelocity/SpeedDownTime;

		}
		Force= Acceleration * m_EntityMass;
	}
	return Force;
}

Vec2d FlightDynamics_2D::GetVelocityFromDistance_Linear(const Vec2d &Distance,const Vec2d &ForceRestraintPositive,const Vec2d &ForceRestraintNegative,double DeltaTime_s, const Vec2d& matchVel)
{
	Vec2d ret;

	if (!m_UsingAccelerationRate)
	{
		ret=__super::GetVelocityFromDistance_Linear(Distance,ForceRestraintPositive,ForceRestraintNegative,DeltaTime_s,matchVel);
		#if 0
		if (m_HeadingToUse)
		{
			Vec2d LocalVelocity=GlobalToLocal(*m_HeadingToUse,m_LinearVelocity);
			if (fabs(m_LinearVelocity[1])>0.0)
				printf("y=%.2f p=%.2f e=%.2f cs=0\n",Distance[1],ret[1],m_LinearVelocity[1]);
		}
		#endif
	}
	else
	{
		const Vec2d MaxDeceleration=GetAcceleration_Delta(1.0,Vec2d(0.0,0.0),false);
#if 0
		//I thought there may be a problem with solving each element separately so I wrote this version
		double dDistance=Distance.length();
		const double RampError=12.0;

		//double AccelerationMagnitude=Acceleration.length();
		double AccelerationMagnitude=ForceRestraintNegative[1]/m_EntityMass; //for now this is the amount of reverse thrust, but I need to work this out at some point

		double Time=sqrt(2.0*dDistance/AccelerationMagnitude);
		ret=(dDistance>RampError)?Distance/max(Time,DeltaTime_s) : Vec2d(0,0,0);

		//DebugOutput("d=%f %f ds=%f s=%f\n",Distance[1],dDistance,ret.length(),m_LinearVelocity.length());

#else
		//Solving each element gives the advantage of using strafe techniques, we are assuming all calls work with local orientation
		for (size_t i=0;i<2;i++)
		{
			double DistanceDirection=Distance[i];
			DistanceDirection-=matchVel[i]*DeltaTime_s;
			if (IsZero(DistanceDirection))
			{
				ret[i]=matchVel[i];
				continue;
			}

			double DistanceLength=fabs(DistanceDirection);

			//Compose our restraint and acceleration based on the component direction
			double Restraint=(DistanceDirection>0)?ForceRestraintPositive[i]:ForceRestraintNegative[i];
			double Acceleration=(Restraint/m_EntityMass);

			double MaxDeceleration_Length=fabs(MaxDeceleration[i]);

			//This number is a work in progress
			const double RampError=2.0;

			{
				//Solve the Acceleration... given a distance what would it have to be going its fastest ramp rate
				double Time=sqrt(2.0*(DistanceLength/MaxDeceleration[i]));
				Acceleration=min(Acceleration,MaxDeceleration_Length*Time);
			}
			double Time=sqrt(2.0*(DistanceLength/Acceleration));

			//TODO at some point this should PID to solve from the super class force feed method... I've put the correct equation below, but it does not work
			//properly with this attempt to solve... these ramp functions are only used in game code so I'll table it for now
			//  [8/26/2012 Terminator]
			#if 1
			//using distance/DTime in the max helps taper off the last frame to not over compensate
			ret[i]=DistanceLength/max(Time,DeltaTime_s);
			#else
			const double MaxSpeed=Acceleration*Time;
			const double IdealSpeed=DistanceLength/DeltaTime_s;
			ret[i]=min(MaxSpeed,IdealSpeed);
			#endif

			//if (i==1)
				//DebugOutput("Distance=%f,Time=%f,Speed=%f,vel=%f\n",DistanceDirection,Time,ret[i],m_LinearVelocity[i]);
				//DebugOutput("Distance=%f,Time=%f,Speed=%f,acc=%f\n",DistanceDirection,Time,ret[i],Acceleration);

			if (DistanceLength>RampError)
			{
				if (DistanceDirection<0)
					ret[i]=-ret[i];
				ret[i]+=matchVel[i];
			}
			else
				ret[i]=matchVel[i];
		}
#endif
	}
	return ret;

}
