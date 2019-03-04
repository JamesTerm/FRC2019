#pragma once

class COMMON_API PhysicsEntity_2D
{
	public:
		PhysicsEntity_2D();
		virtual ~PhysicsEntity_2D() {}
		
		///You must set the mass otherwise you will get the default
		void SetMass(double mass);
		double GetMass() const;
		///This will zero out all vectors
		virtual void ResetVectors();

		// An overloaded operator for matching the current physics and position
		virtual void CopyFrom(const PhysicsEntity_2D& rhs);

		///This will compute all the displacement. Call this each time slice to obtain the rotation and position displacements.  
		///You must call this to run the physics engine!
		virtual void TimeChangeUpdate(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement);

		///This simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		///This is ideal to be used with GetTorqueFromVelocity
		/// \param normalize will normalize the distance between -pi - pi
		double GetVelocityFromDistance_Angular(double Distance,double Restraint,double DeltaTime_s, double matchVel,bool normalize=true);

		inline Vec2D GetVelocityFromCollision(Vec2D ThisVelocityToUse,double otherEntityMass,Vec2D otherEntityVelocity);
		///This simply returns a min operation of speed/time and the maximum speed available to stop within the given distance
		///This can either work with local or global orientation that all depends on the orientation of the restraints typically this works in local
		virtual Vec2D GetVelocityFromDistance_Linear(const Vec2D &Distance,const Vec2D &ForceRestraintPositive,const Vec2D &ForceRestraintNegative,double DeltaTime_s, const Vec2D& matchVel);
		virtual Vec2D GetVelocityFromDistance_Linear_v1(const Vec2D &Distance,const Vec2D &ForceRestraintPositive,const Vec2D &ForceRestraintNegative,double DeltaTime_s, const Vec2D& matchVel);

		///These are coefficients to use when dealing with a force of friction typically 0.8 and 0.2 respectively
		void SetFriction(double StaticFriction, ///<The amount of friction to be applied when object is not moving
			double KineticFriction ///<The amount of friction to be applied when object is moving
			);

		///This is the direct way to handle torque for various mass distributions
		///Here are some examples:
		/// - Disk rotating around its center				0.5
		/// - Hollow cylinder rotating around its center	1.0
		/// - Hollow sphere									0.66 (2/3)
		/// - Hoop rotating around its center				1.0
		/// - Point mass rotating at radius r				1.0
		/// - Solid cylinder								0.5
		/// - Solid sphere									0.4
		/// \todo Provide helper methods to compute this value
		void SetAngularInertiaCoefficient(double AngularInertiaCoefficient);
		///This sets the radius for yaw axis, pitch axis, and roll axis.  Default = 1,1,1
		void SetRadiusOfConcentratedMass(double RadiusOfConcentratedMass);
		double GetRadiusOfConcentratedMass() const;

		void SetLinearVelocity( const Vec2D &LinearVelocity);
		Vec2D GetLinearVelocity() const;

		void SetAngularVelocity( double AngularVelocity);
		double GetAngularVelocity() const;

		//This will give the acceleration delta given the torque which is: torque / AngularInertiaCoefficient * Mass
		inline double GetAngularAccelerationDelta(double torque,double RadialArmDistance=1.0);

		///These will auto sum for each call made, the forces last for one second during each timer update, so you have to call them repeatedly to 
		///continue to apply force.  If you want to apply a force for a specific amount of time, this can be achieved by calling this
		///during each time slice.  
		/// \note This should offer enough precision in time, but you can get more precise by computing a fraction force if necessary
		//void ApplyForce( const Vec2D &force);
		//void ApplyTorque( double torque);

		///These work like the above except that the force applied only happens for a fraction of a second.  For accurate results FrameDuration
		///should be <= 1/framerate.  Ideally these should be used for high precision movements like moving a ship, where the FrameDuration is
		///typically the TimeDelta value
		void ApplyFractionalForce( const Vec2D &force,double FrameDuration);
		void ApplyFractionalTorque( double torque,double FrameDuration,double RadialArmDistance=1.0);

		///This one is ideal to use for collision detection.  It will basically evaluate the point and determine the amount of force and torque to
		///apply.  It will implicitly call ApplyFractionalForce and ApplyFractionalTorque.
		void ApplyFractionalForce( const Vec2D &force, const Vec2D &point,double FrameDuration );

		///You may prefer to set a desired speed instead of computing the forces.  These values returned are intended to be used with 
		///ApplyFractionalForce, and ApplyFractionalTorque respectively.
		/// For the force, this works with the current linear velocity; therefore the desired velocity and return must work in global orientation
		virtual Vec2D GetForceFromVelocity( 
			const Vec2D &vDesiredVelocity,	///< How fast do you want to go (in a specific direction)
			double DeltaTime_s					///< How quickly do you want to get there
			) const;
		virtual double GetTorqueFromVelocity( 
			double vDesiredVelocity,	///< How fast do you want to go (in a specific direction)
			double DeltaTime_s					///< How quickly do you want to get there (usually in time slices)
			) const;

		///This is a clean way to compute how much torque that can be applied given the maximum amount available (e.g. thruster capacity)
		///It should be noted that this treats roll as a separate factor, which is best suited for avionic type of context
		/// \Note all restraint parameters are positive (i.e. ForceRestraintNegative)
		double ComputeRestrainedTorque(double Torque,double TorqueRestraint,double dTime_s);
		Vec2D ComputeRestrainedForce(const Vec2D &LocalForce,const Vec2D &ForceRestraintPositive,const Vec2D &ForceRestraintNegative,double dTime_s);
		//This returns in the form of magnitude using the proper equations
		static double GetCentripetalAcceleration(double LinearVelocity,double AngularVelocity,double DeltaTime_s);
		double GetCentripetalAcceleration(double DeltaTime_s) const;
		///This returns the global acceleration needed to maintain linear velocity
		Vec2D GetCentripetalAcceleration_2D(double DeltaTime_s) const;
		///This returns the global force needed to maintain the current linear velocity
		Vec2D GetCentripetalForce(double DeltaTime_s) const;
	protected:
		double m_EntityMass;
		double m_StaticFriction,m_KineticFriction;

		double m_AngularInertiaCoefficient;
		double m_RadiusOfConcentratedMass; //This is used to compute the moment of inertia for torque (default 1,1,1)

		Vec2D m_LinearVelocity;		///< This must represent global orientation for external forces to work properly
		double m_AngularVelocity;		///< All angle variables are in radians!

		///This variable is factored in but is managed externally 
		Vec2D m_SummedExternalForces;
		double m_lastTime_s;
};

///This class is a expands on some common tasks that deal more specifically with flight.  This attempts to pull common tasks needed from physics in a way
///Where it is easy to use for ships and other objects that deal with orientation and position
class COMMON_API FlightDynamics_2D : public PhysicsEntity_2D
{
	public:
		//provide common area to initialize members
		void init();
		FlightDynamics_2D();
		FlightDynamics_2D(const double *HeadingToUse);
		FlightDynamics_2D(const double &HeadingToUse);
		virtual ~FlightDynamics_2D() {}

		//Allow late binding of heading to use (client code should set this once as soon as possible)
		void SetHeadingToUse(const double *HeadingToUse) {m_HeadingToUse=HeadingToUse;}
		virtual void ResetVectors();

		///This will measure the distance between this quat and the look dir quat.  With the current algorithm the most desired results occur when the
		///delta's are less than 90 degrees for yaw and pitch.  Roll is computed separately.
		/// \param lookDir This is another orientation that you are comparing against
		/// \param UpDir is usually the same quat's orientation * 0,0,1
		Vec2D ComputeAngularDistance_asLookDir(const Vec2D &lookDir);
		double ComputeAngularDistance(const Vec2D &lookDir);
		/// \param Orientation this will break down the quat into its lookDir and UpDir for you
		//Vec2D ComputeAngularDistance(double Orientation);
		double ComputeAngularDistance(double Orientation);
		const double &GetHeading() {return *m_HeadingToUse;}

		virtual void TimeChangeUpdate(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement);

		//Acceleration rate methods:

		///If these methods are being used, this must be set to true
		void SetUsingAccelerationRate(bool UseIt) {m_UsingAccelerationRate=UseIt;} 
		struct LinearAccelerationRates
		{
			Vec2D AccDeltaPos;        //when increasing from a positive position
			Vec2D AccDeltaNeg;        //when -increasing from a negative position
			Vec2D DecDeltaPos;        //when decreasing from a positive position
			Vec2D DecDeltaNeg;        //when -decreasing from a negative position
		};
		///Get and set the linear acceleration rates here.
		///For now this is the parameters for the simple model, which may or may not be applied for other engine models
		///Typically this should be a one time setup, but can be dynamic (e.g. afterburner)
		LinearAccelerationRates &GetLinearAccelerationRates();
		//Set to the desired acceleration level.  This should be called first per time interval, so the other methods can work properly
		//This must use local orientation
		void SetTargetAcceleration(const Vec2D &TargetAcceleration);
		///This can be defined to just about anything, but to keep things simple it is a linear constant depending on direction
		///We could later have different models and have client choose which to use
		Vec2D GetAcceleration_Delta (double dTime_s);
		///With this version you may use to predict the capabilities of max acceleration or deceleration
		/// \param Clipping this chooses whether to extract the full power when the acceleration approaches target acceleration, or
		/// whether to compute the actual force necessary to hit target.  Typically this will be on when manipulating the force
		/// The no clip is useful for things which need know full capability to get to threshold such as in GetForceFromVelocity
		Vec2D GetAcceleration_Delta (double dTime_s,const Vec2D &TargetAcceleration,bool Clipping=true);
		///This applies the current acceleration delta (i.e. calls GetAcceleration_Delta) to acceleration.
		/// \note Special care has been taken to ensure that any reads to GetAcceleration_Delta will be the same value applied here.
		///This must be consistent per time slice so that other clients can properly predict the acceleration.
		void Acceleration_TimeChangeUpdate(double dTime_s);
		///Read-only access of the current acceleration
		const Vec2D &GetCurrentAcceleration() {return m_CurrentAcceleration;}

		/// These are overloaded to optionally factor in the acceleration period
		virtual Vec2D GetForceFromVelocity(const Vec2D &vDesiredVelocity,double DeltaTime_s);
		virtual Vec2D GetVelocityFromDistance_Linear(const Vec2D &Distance,const Vec2D &ForceRestraintPositive,const Vec2D &ForceRestraintNegative,double DeltaTime_s, const Vec2D& matchVel);
		
		double StructuralDmgGLimit;
		double G_Dampener;

	private:
		#ifndef _Win32
		typedef PhysicsEntity_2D __super;
		#endif
		double m_DefaultHeading;
		// I'll try to keep this read only, so that client who own their own Heading can use this code, without worrying about the Heading being changed
		const double *m_HeadingToUse;
		//This keeps track of the current rate of acceleration.  These are in local orientation.
		Vec2D m_CurrentAcceleration,m_TargetAcceleration;
		LinearAccelerationRates m_LinearAccelerationRates;
		//Some objects may not need to use this (by default they will not)
		bool m_UsingAccelerationRate;
};
