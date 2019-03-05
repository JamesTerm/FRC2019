#pragma once

class Vehicle_Drive_Common_Interface
{
	public:
		//override the wheel dimensions, which by default are the entities dimensions (a good approximation)
		virtual const Vec2D &GetWheelDimensions() const =0;
		//This returns the measurement of the turning diameter where the wheels turn within themselves
		//usually for a 4 wheel drive this is length from corner to corner, and for a 6 wheel drive this ranges from the track width (if centered)
		//to either set of 4 wheels (sharing the same center).  Typically the set of 4 wheels makes a square resulting in track width
		virtual double GetWheelTurningDiameter() const =0;
		virtual double Vehicle_Drive_GetAtt_r() const=0;
		virtual const PhysicsEntity_2D &Vehicle_Drive_GetPhysics() const=0;
		//This method will help me identify write operation cases easier
		virtual PhysicsEntity_2D &Vehicle_Drive_GetPhysics_RW()=0;
};

//The way the drive works is like the following:
//1.  Get desired velocity (e.g. from distance)
//2.  Update Voltage
//3.  PID
//4.  Translate velocity
//5.  Inject displacement
class Vehicle_Drive_Common
{
	public:
		Vehicle_Drive_Common(Vehicle_Drive_Common_Interface *VehicleProps);
		virtual ~Vehicle_Drive_Common() {}
		virtual void InterpolateThrusterChanges(Vec2D &LocalForce,double &Torque,double dTime_s);
		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s)=0;

		//This will convert the force into both motor velocities and interpolate the final torque and force to apply
		//Unlike in robot tank We'll only cache the values to work with in the Apply Thrusters, and apply them only to the inject displacement
		//This way when swerve is unable to deliver due to error and limitations, the actual control will not be compromised
		void Vehicle_Drive_Common_ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
		bool Vehicle_Drive_Common_InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement);
		//TODO verify this is not needed
		//const Vec2D &GetCachedLocalForce() {return m_CachedLocalForce;}
	protected:
		Vehicle_Drive_Common_Interface * const m_VehicleProps;
	private:
		Vec2D m_CachedLocalForce,m_CachedLinearVelocity;
		double m_CachedTorque,m_CachedAngularVelocity;
};

///This kind of ship will convert the torque and force into a two fixed point force/thrust system like that of a tank.  It also interpolates the left
///right velocities back into the torque and force.  Unlike a ship, it will always absorb any lateral forces as it is assumed that the entity will not
///skid in those directions.  This means it cannot strafe, and will behave much like a vehicle on land rather than a ship in space.
class COMMON_API Tank_Drive : public Vehicle_Drive_Common
{
	private:
		Vehicle_Drive_Common_Interface * const m_pParent;
		#ifndef _Win32
		typedef Vehicle_Drive_Common __super;
		#endif
		double m_LeftLinearVelocity,m_RightLinearVelocity;
	public:
		Tank_Drive(Vehicle_Drive_Common_Interface *Parent);
		double GetLeftVelocity() const {return m_LeftLinearVelocity;}
		double GetRightVelocity() const {return m_RightLinearVelocity;}
		// Places the ship back at its initial position and resets all vectors
		virtual void ResetPos();

		//This method converts the given left right velocities into a form local linear velocity and angular velocity
		void InterpolateVelocities(double LeftLinearVelocity,double RightLinearVelocity,Vec2D &LocalVelocity,double &AngularVelocity,double dTime_s);
		//Overload this for optimal time between the update and position to avoid oscillation
		virtual void InterpolateThrusterChanges(Vec2D &LocalForce,double &Torque,double dTime_s);
		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s);

		//This will convert the force into both motor velocities and interpolate the final torque and force to apply
		virtual void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
		virtual bool InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement);
};

struct SwerveVelocities
{
	enum SectionOrder
	{
		eFrontLeft,
		eFrontRight,
		eRearLeft,
		eRearRight
	};
	SwerveVelocities()
	{
		memset(this,0,sizeof(SwerveVelocities));
	}
	union uVelocity
	{
		struct Explicit
		{
			double sFL,sFR,sRL,sRR; //wheel tangential speeds in MPS
			double aFL,aFR,aRL,aRR; //wheel angles in radians clockwise from straight ahead
		} Named;
		double AsArray[8];
	} Velocity;
};

class Swerve_Drive_Interface : public Vehicle_Drive_Common_Interface
{
	public:
	virtual const SwerveVelocities &GetSwerveVelocities() const =0;
	//This is really for butterfly and nona drive, but put here to greatly simplify the code
	virtual bool IsTractionMode() const {return false;}
};

class COMMON_API Swerve_Drive : public Vehicle_Drive_Common
{
	public:
		Swerve_Drive(Swerve_Drive_Interface *Parent);
		// Places the ship back at its initial position and resets all vectors
		virtual void ResetPos();

		double GetIntendedVelocitiesFromIndex(size_t index) const; //This is sealed always using m_Velocities
		double GetSwerveVelocitiesFromIndex(size_t index) const; //This is sealed always using m_Velocities
		//Overload this for optimal time between the update and position to avoid oscillation
		virtual void InterpolateThrusterChanges(Vec2D &LocalForce,double &Torque,double dTime_s);
		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s);

		virtual void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
		virtual bool InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement);

		//This method converts the given left right velocities into a form local linear velocity and angular velocity
		virtual void InterpolateVelocities(const SwerveVelocities &Velocities,Vec2D &LocalVelocity,double &AngularVelocity,double dTime_s);
	protected:
		Swerve_Drive_Interface * const m_pParent;
		#ifndef _Win32
		typedef Vehicle_Drive_Common __super;
		#endif
		SwerveVelocities m_Velocities;
		const SwerveVelocities &GetIntendedVelocities() const {return m_Velocities;}
};

class COMMON_API Butterfly_Drive : public Swerve_Drive
{
	public:
		Butterfly_Drive(Swerve_Drive_Interface *Parent);

		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s);
		virtual void InterpolateVelocities(const SwerveVelocities &Velocities,Vec2D &LocalVelocity,double &AngularVelocity,double dTime_s);
	protected:
		virtual double GetStrafeVelocity(const PhysicsEntity_2D &PhysicsToUse,double dTime_s) const;
		Vec2D m_GlobalStrafeVelocity; //Cache the strafe velocity in its current direction
	    Vec2D m_LocalVelocity;  //keep track of the current velocity (for better interpreted displacement of strafing)
		Vec2D m_PreviousGlobalVelocity;
		void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
};

class COMMON_API Nona_Drive : public Butterfly_Drive
{
	private:
		#ifndef _Win32
		typedef Butterfly_Drive __super;
		#endif
		double m_KickerWheel;
	public:
		Nona_Drive(Swerve_Drive_Interface *Parent);

		virtual void UpdateVelocities(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double Torque,double TorqueRestraint,double dTime_s);
		double GetKickerWheelIntendedVelocity() const {return m_KickerWheel;}
		//This is used as a cheat to avoid needing to resolve velocity using a callback technique... this needs to be set back to what it was when
		//altered
		void SetKickerWheelVelocity(double velocity) {m_KickerWheel=velocity;}
	protected:
		virtual double GetStrafeVelocity(const PhysicsEntity_2D &PhysicsToUse,double dTime_s) const;
		void ApplyThrusters(PhysicsEntity_2D &PhysicsToUse,const Vec2D &LocalForce,double LocalTorque,double TorqueRestraint,double dTime_s);
};

