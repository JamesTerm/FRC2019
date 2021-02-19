#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Base/PIDController.h"

#include "../DriveKinematics/Vehicle_Drive.h"
#include "../../../Properties/RegistryV1.h"
#include "SimulatedOdometry.h"
#include "../../../Base/Physics.h"

#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _Simulation_
class Potentiometer_Tester4
{
#pragma region _Description_
	//This is specialized for motor simulation turning a loaded mass
	//This version is much simpler than the others in that it works with the motor curves to determine 
	//This will model a solid sphere for the moment of inertia
#pragma endregion
private:
	#pragma region _members_
	Framework::Base::PhysicsEntity_1D m_motor_wheel_model;
	double m_free_speed_rad = 18730 * (1.0 / 60.0) * Pi2; //radians per second of motor
	double m_stall_torque = 0.71;  //Nm
	double m_gear_reduction = (1.0 / 81.0) * (3.0 / 7.0);
	//This will account for the friction, and the load torque lever arm
	double m_gear_box_effeciency = 0.65;
	double m_mass = Pounds2Kilograms(1.0);  //just motor
	double m_RadiusOfConcentratedMass = Inches2Meters(1.12);  //just the motor's diameter
	double m_AngularInertiaCoefficient = 0.5;  //solid cylinder
	//The dead zone defines the opposing force to be added to the mass we'll clip it down to match the velocity
	double m_dead_zone = 0.10; //this is the amount of voltage to get motion can be tested
	double m_anti_backlash_scaler = 0.85; //Any momentum beyond the steady state will be consumed 1.0 max is full consumption
	double m_Time_s = 0.010;
	double m_Heading = 0.0;
	size_t m_InstanceIndex = 0;  //for ease of debugging
	#pragma endregion
public:
	void Initialize(size_t index, const Framework::Base::asset_manager* props = NULL,
		const char* prefix = properties::registry_v1::csz_CommonSwivel_)
	{
		if (props)
		{
			using namespace ::properties::registry_v1;
			std::string constructed_name;
			#define GET_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = props->get_number(constructed_name.c_str(), y);

			GET_NUMBER(Pot4_free_speed_rad, m_free_speed_rad);
			GET_NUMBER(Pot4_stall_torque_NM, m_stall_torque);
			GET_NUMBER(Pot4_motor_gear_reduction, m_gear_reduction);
			GET_NUMBER(Pot4_gear_box_effeciency, m_gear_box_effeciency);
			GET_NUMBER(Pot4_mass, m_mass);
			GET_NUMBER(Pot4_RadiusOfConcentratedMass, m_RadiusOfConcentratedMass);
			GET_NUMBER(Pot4_AngularInertiaCoefficient, m_AngularInertiaCoefficient);
			GET_NUMBER(Pot4_dead_zone, m_dead_zone);
			GET_NUMBER(Pot4_anti_backlash_scaler, m_anti_backlash_scaler);
			//GET_NUMBER();
			#undef GET_NUMBER
		}
		m_motor_wheel_model.SetMass(m_mass);
		m_motor_wheel_model.SetAngularInertiaCoefficient(m_AngularInertiaCoefficient);
		m_motor_wheel_model.SetRadiusOfConcentratedMass(m_RadiusOfConcentratedMass);
		m_InstanceIndex = index;
	}
	double GetTorqueFromVoltage(double Voltage)
	{
		//Compute the torque to apply from the speed of a scaled voltage curve of a scaled max torque; 
		//Note: the motor curve max torque is scaled
		//Important: The only one voltage factor here will establish the direction of torque
		const double max_torque = m_stall_torque * Voltage;
		//The speed ratio is a ratio of speed to "max speed" of a voltage scaled motor curve and must only be magnitude
		const double speed_ratio = fabs(Voltage) > 0.0 ?
			std::max(1.0 - (fabs(m_motor_wheel_model.GetVelocity()) / (m_free_speed_rad * fabs(Voltage))), 0.0) : 0.0;
		const double torque = max_torque * speed_ratio;
		return torque;
	}
	double GetMechanicalRestaintTorque(double Voltage, double next_current_velocity)
	{
		double adverse_torque = 0.0;
		const double max_torque = m_stall_torque;
		//Compute adverse torque as the equilibrium sets in to no motion, note the current velocity is motor velocity (no reduction)
		//as this will be easier to read
		if (next_current_velocity != 0)
		{
			//Don't have this mess with small increments, friction will take the final bite here
			if (fabs(next_current_velocity) < 0.01)
			{
				m_motor_wheel_model.ResetVectors();
			}
			else if (m_Time_s > 0.0)  // no division by zero
			{
				//May want to provide properties to control how much dead-zone torque to blend
				//Found a straight constant gives correct result, but have to adjust mass to get correct latency
				#if 0
				const double dead_zone_torque = m_dead_zone * max_torque;
				adverse_torque = (1.0 - fabs(0.6*Voltage)) * dead_zone_torque + (0.4*Voltage)* dead_zone_torque; //this is the minimum start
				#else
				adverse_torque = (m_dead_zone * max_torque); //this is the minimum start
				#endif

				//Evaluate anti backlash, on deceleration the momentum of the payload that exceeds the current steady state
				//will be consumed by the gearing, we apply a scaler to module the strength of this
				//Note: this could work without reduction if next_current_velocity didn't have it as well, but it is easier to read with it in
				const double steady_state_velocity = Voltage * m_free_speed_rad; //for now not factoring in efficiency
				//both the voltage and velocity must be in the same direction... then see if we have deceleration
				if ((steady_state_velocity * next_current_velocity > 0.0) && (fabs(next_current_velocity) > fabs(steady_state_velocity)))
				{
					//factor all in to evaluate note we restore direction at the end
					const double anti_backlash_accel = (fabs(next_current_velocity) - fabs(steady_state_velocity)) / m_Time_s;
					const double anti_backlash_torque = m_motor_wheel_model.GetMomentofInertia() * anti_backlash_accel;
					//Backlash only kicks in when the excess momentum exceed the dead zone threshold
					adverse_torque += anti_backlash_torque * m_anti_backlash_scaler;
				}
				//just compute in magnitude, then restore the opposite direction of the velocity
				const double adverse_accel_limit = fabs(next_current_velocity) / m_Time_s; //acceleration in radians enough to stop motion
				//Now to convert into torque still in magnitude
				const double adverse_torque_limit = m_motor_wheel_model.GetMomentofInertia() * adverse_accel_limit;
				//clip adverse torque as-needed and restore the opposite sign (this makes it easier to add in the next step
				adverse_torque = std::min(adverse_torque_limit, adverse_torque) * ((next_current_velocity > 0) ? -1.0 : 1.0);
			}
		}
		return adverse_torque;
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		//Note: these are broken up for SwerveEncoders_Simulator4 to obtain both torques before applying them
		const double voltage_torque = GetTorqueFromVoltage(Voltage);
		m_motor_wheel_model.ApplyFractionalTorque(voltage_torque, m_Time_s);
		const double adverse_torque = GetMechanicalRestaintTorque(Voltage, m_motor_wheel_model.GetVelocity());
		m_motor_wheel_model.ApplyFractionalTorque(adverse_torque, m_Time_s);
	}
	double GetPotentiometerCurrentPosition() const
	{
		return m_Heading;
	}
	void TimeChange()
	{
		double PositionDisplacement;
		m_motor_wheel_model.TimeChangeUpdate(m_Time_s, PositionDisplacement);
		//No normalizing here, we represent the actual position!
		m_Heading += PositionDisplacement * m_gear_reduction;  //velocity is motor, so convert to output rate with the gear reduction
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s)
	{
		m_Time_s = dTime_s;
	}
	void ResetPos()
	{
		m_motor_wheel_model.ResetVectors();
	}
	Framework::Base::PhysicsEntity_1D& GetWheelModel_rw()
	{
		return m_motor_wheel_model;
	}
	double GetMaxSpeed(bool with_reduction = false) const
	{
		return m_free_speed_rad * (with_reduction ? m_gear_reduction : 1.0);
	}
	double GetGearReduction() const
	{
		return m_gear_reduction;
	}
	double GetGearBoxEffeciency() const
	{
		return m_gear_box_effeciency;
	}
};

class SwerveEncoders_Simulator4
{
#pragma region _Description_
	//This works on the same motor concepts as Potentiometer_Tester4 to compute the torque out, but there is an additional load working 
	//against the motor, to work out what this is the interface mechanism will be cleaner to write by providing all at the same time
	//Ultimately we can make use of Torque = F X R equation and apply this to the mass where R is the wheel radius.  To properly simulate
	//We need to move a 2D mass (using 2D physics) and then break down the vectors that are n line with the wheel, and apply.  This can go
	//in either direction.  In the case of opposing torques the magnitude of the forces (before they cancel each other) must be less than
	//weight * static CoF.  The force is torque / wheel radius and the acceleration is known from the torque applied if this is greater
	//clamp down torque to max and put in kinetic friction mode while the forces are clamped.  When it kinetic mode the torque is detached
	//somewhat where the limit is lower (that goes for how much torque can be transferred to mass as well.  Once the forces subside to the
	//lower limit (weight * kinetic CoF) this can flip it back to static CoF mode.
#pragma endregion
private:
	#pragma region _member variables_
	Potentiometer_Tester4 m_Encoders[4];
	Framework::Base::PhysicsEntity_2D m_Payload;
	Framework::Base::PhysicsEntity_1D m_Friction;  //apply friction to skid
	std::function< SwerveVelocities&()> m_CurrentVelocities_callback;
	std::function<SwerveVelocities()> m_VoltageCallback;
	Inv_Swerve_Drive m_Input;
	double m_KineticFriction = 0.50; //this looks about right, but we can try to fix
	class SwerveDrive_Plus : public Swerve_Drive
	{
	private:
		SwerveVelocities m_FromExistingOutput;
	public:
		const SwerveVelocities &UpdateFromExisting(double FWD, double STR, double RCW,const SwerveVelocities &CurrentVelocities)
		{
			using namespace Framework::Base;
			//Essentially there is a common vector that has to work with the wheel angles from current velocities. Because the wheel 
			//angles can be reversed we have to approach this per each wheel angle.  The result is that the magnitude that occupies the
			//same direction will be preserved, and the rest will be discarded to friction. We'll also have to be mindful the direction
			//may be opposite in which case we reverse the sign to keep the same direction.
			//First we let the kinematics we inherit from work like before, and then work from those quad vectors
			Swerve_Drive::UpdateVelocities(FWD, STR, RCW);
			//If the wheel angles are not know, then grab then grab them from existing
			if (IsZero(fabs(FWD) + fabs(STR) + fabs(RCW)))
			{
				for (size_t i = 0; i < 4; i++)
					SetVelocitiesFromIndex(i + 4, CurrentVelocities.Velocity.AsArray[i + 4]);
			}
			for (size_t i = 0; i < 4; i++)
			{
				//This next part is tricky... we take our projected vector and rotate it to a global view using the direction of our actual
				//wheel angle.  Everything that exists in the Y component will be what we consume... X will be discarded
				const Vec2D projected_vector_local(0,GetIntendedVelocitiesFromIndex(i));  //local magnitude pointing up
				const double projected_vector_heading = GetSwerveVelocitiesFromIndex(i);
				//point it in it's current direction
				const Vec2D projected_vector_global = LocalToGlobal(projected_vector_heading,projected_vector_local);
				//Point it back locally to the direction of our actual wheel angle
				const double actual_vector_heading = CurrentVelocities.Velocity.AsArray[i + 4];
				const Vec2D actual_vector_local = GlobalToLocal(actual_vector_heading, projected_vector_global);
				//TODO see if there is ever a need to reverse the direction, there is a case where 2 wheels on one side are
				//opposite causing the velocity to go half speed (not sure if that is here though)
				const double IsReversed = 1.0;

				//We have our velocity in the Y component, now to evaluate if it needs to be reversed
				//const double IsReversed = fabs(projected_vector_heading) - fabs(actual_vector_heading) > Pi / 2 ? -1.0 : 1.0;

				//Now to populate our result
				m_FromExistingOutput.Velocity.AsArray[i] = actual_vector_local.y() * IsReversed;
				m_FromExistingOutput.Velocity.AsArray[i + 4] = CurrentVelocities.Velocity.AsArray[i + 4];  //pedantic
			}
			return m_FromExistingOutput;
		}
	};
	SwerveDrive_Plus m_Output;
	Swerve_Drive m_TestForce; //used to measure acceleration
	double m_WheelDiameter_In=4.0;
	bool m_IsSkidding=false; //cache last state to determine or force normal
	#pragma endregion
public:
	virtual void Initialize(const Framework::Base::asset_manager* props = NULL)
	{
		//TODO get properties for our local members
		m_Input.Init(props);
		m_TestForce.Init(props);
		m_Output.Init(props);
		Framework::Base::asset_manager DefaultMotorProps;
		using namespace properties::registry_v1;
		
		if (props)
		{
			m_WheelDiameter_In = props->get_number(csz_Drive_WheelDiameter_in,m_WheelDiameter_In);
			//This loads up the defaults first and then any scripted override
			for (size_t i = 0; i < 4; i++)
			{
				m_Encoders[i].Initialize(i, &DefaultMotorProps, csz_CommonDrive_);
				m_Encoders[i].Initialize(i, props, csz_CommonDrive_);
			}
		}
		else
		{
			//We only have defaults
			for (size_t i = 0; i < 4; i++)
				m_Encoders[i].Initialize(i, &DefaultMotorProps, csz_CommonDrive_);
		}
		//TODO: We may want to add property to the mass
		m_Payload.SetMass(Pounds2Kilograms(148));
		m_Friction.SetMass(Pounds2Kilograms(148));
		//May want friction coefficients as well
		m_Friction.SetStaticFriction(1.0);  //rated from wheels
		m_Friction.SetKineticFriction(m_KineticFriction);  //not so easy to measure
	}
	void SetVoltageCallback(std::function<SwerveVelocities()> callback)
	{
		m_VoltageCallback = callback;
	}
	void TimeChange(double dTime_s)
	{
		using namespace Framework::Base;
		//Make sure the potentiometer angles are set in current velocities before calling in here
		SwerveVelocities InitialVelocitiesForPayload = m_CurrentVelocities_callback();
		for (size_t i = 0; i < 4; i++)
		{
			const double Voltage = m_VoltageCallback().Velocity.AsArray[i];
			Potentiometer_Tester4& encoder = m_Encoders[i];
			encoder.UpdatePotentiometerVoltage(Voltage);
			//This is our initial velocity to submit to payload
			const double LinearVelocity = encoder.GetWheelModel_rw().GetVelocity() * encoder.GetGearReduction() *
				Inches2Meters(m_WheelDiameter_In * 0.5);

			#if 0
			m_CurrentVelocities_callback().Velocity.AsArray[i] = LinearVelocity;
			return;
			#else
			InitialVelocitiesForPayload.Velocity.AsArray[i] = LinearVelocity;
			#endif
		}
		//Now we can interpolate the velocity vectors with the same kinematics of the drive
		m_Input.InterpolateVelocities(InitialVelocitiesForPayload);

		//printf("force=%.2f\n",ForcesForPayload.Velocity.AsArray[0]);
		//Before we apply the forces grab the current velocities to compute the new forces
		const Vec2D last_payload_velocity = m_Payload.GetLinearVelocity();
		const double last_payload_angular_velocity = m_Payload.GetAngularVelocity();
		const double drive_train_efficiency = m_Encoders[0].GetGearBoxEffeciency();
		//Now to convert the inputs velocities into force, which is the velocity change (per second) and divide out the time
		//to get our acceleration impulse, finally multiple by mass for the total.
		//Note: I may need to consider conservation of momentum here, but for now scaling down using drive train efficiency does add the latency
		Vec2D InputAsForce = (Vec2D(m_Input.GetLocalVelocityX(), m_Input.GetLocalVelocityY()) - last_payload_velocity) / dTime_s * 
			m_Payload.GetMass() * drive_train_efficiency;
		double InputAsTorque = (m_Input.GetAngularVelocity() - last_payload_angular_velocity) / dTime_s * 
			m_Payload.GetMomentofInertia() * drive_train_efficiency;

		double summed_y_force = 0.0;
		//see if we are decelerating
		const Vec2D current_local_velocity(m_Input.GetLocalVelocityX(), m_Input.GetLocalVelocityY());
		const double current_local_speed = current_local_velocity.length();  //aka magnitude
		const double last_payload_speed = last_payload_velocity.length();
		bool IsDecelerating = current_local_speed < last_payload_speed;
		if (IsDecelerating)
		{
			//Make sure the rotation isn't overpowering this indication
			if (fabs(m_Input.GetAngularVelocity()) > fabs(last_payload_angular_velocity))
			{
				//compare apples to apples
				const double angular_velocity_asLinear = m_Input.GetAngularVelocity() * Pi * m_Input.GetDriveProperties().GetTurningDiameter();
				if (fabs(angular_velocity_asLinear) > current_local_speed)
					IsDecelerating = false;
			}
		}
		if ((!IsDecelerating) && (fabs(m_Input.GetAngularVelocity()) < fabs(last_payload_angular_velocity)))
		{
			IsDecelerating = true; //kind of redundant but makes the logic a mirror reflection of above
			if (current_local_speed > last_payload_speed)
			{
				//compare oranges to oranges
				const double angular_velocity_asLinear = m_Input.GetAngularVelocity() * Pi * m_Input.GetDriveProperties().GetTurningDiameter();
				if (current_local_speed > fabs(angular_velocity_asLinear))
					IsDecelerating = false;
			}
		}

		if (!IsDecelerating)
		{
			//Testing the total force start by adding up the total velocity delta
			m_TestForce.UpdateVelocities(m_Input.GetLocalVelocityY() - last_payload_velocity.y(), 
				m_Input.GetLocalVelocityX() - last_payload_velocity.x(), m_Input.GetAngularVelocity() - last_payload_angular_velocity);
			double max_wheel_velocity=0.0;
			for (size_t i = 0; i < 4; i++)
				if (m_TestForce.GetIntendedVelocitiesFromIndex(i) > max_wheel_velocity)
					max_wheel_velocity = m_TestForce.GetIntendedVelocitiesFromIndex(i);
			//sum all 4 wheels... if one of them exceeds force they all do
			summed_y_force = max_wheel_velocity / dTime_s * m_Payload.GetMass() * drive_train_efficiency;
			//printf("|sum=%.2f,y=%.2f|",summed_y_force,InputAsForce.y());
		}
		//Now we can easily evaluate the y-component of force to see if we have a skid, for now we only care about linear motion
		//as rotation with motion only impacts the x-component of the wheels and is addressed below
		//The spin in place may be added here later, but leaving out for now as this is not a typical stress that needs to be simulated
		const bool IsSkidding = fabs(summed_y_force) > m_Friction.GetForceNormal(g,m_IsSkidding?1:0);
		const int FrictionMode = IsSkidding ? 1 : 0;
		//Scale down even further if we are skidding
		if (IsSkidding)
		{
			#if 0
			static int counter = 0;
			printf("[%d]skid [%.2f>%.2f]\n",counter++,InputAsForce.y(),m_Friction.GetForceNormal(g, m_IsSkidding ? 1 : 0));
			#endif
			//the scaled amount is fudged to what looks about right since the summed forces is not quite accurate
			const double scale = m_Friction.GetForceNormal(g, m_IsSkidding ? 1 : 0) / std::max(fabs(summed_y_force)*2.0,m_KineticFriction);
			//This may be a bit more scale than in reality but should be effective
			InputAsForce *= scale;
			InputAsTorque *= scale;
		}
		m_IsSkidding = IsSkidding; //stays skid until we slow down enough below the kinetic friction

		//Apply our torque forces now, before working with friction forces
		//Note: each vector was averaged (and the multiply by 0.25 was the last operation)
		m_Payload.ApplyFractionalForce(InputAsForce, dTime_s);
		m_Payload.ApplyFractionalTorque(InputAsTorque, dTime_s);
		//printf("|pay_t=%.2f|", m_Input.GetAngularVelocity());

		//Check direction, while the controls may not take centripetal forces into consideration, or if in open loop and no feedback
		//It is possible to skid, where the intended direction of travel doesn't match the existing, to simulate, we localize the
		//the existing velocity to the new intended direction of travel, and then apply friction force to the x component which is force normal
		//by the kinetic friction coefficient.  This has to clip as the x components velocity reaches zero
		{
			const Vec2D IntendedVelocities = Vec2D(m_Input.GetLocalVelocityX(),m_Input.GetLocalVelocityY());
			Vec2D IntendedVelocities_normalized(m_Input.GetLocalVelocityX(), m_Input.GetLocalVelocityY());
			double magnitude = IntendedVelocities_normalized.normalize();
			//Use this heading to rotate our velocity to global
			const double IntendedDirection=atan2(IntendedVelocities_normalized[0], IntendedVelocities_normalized[1]);
			const Vec2D local_payload_velocity = m_Payload.GetLinearVelocity();
			//Note: aligned_to_Intended this is the current velocity rotated to align with the intended velocity, when the current velocity direction
			//is mostly the same as the intended, the Y component will consume it.
			const Vec2D aligned_to_Intended_velocity = GlobalToLocal(IntendedDirection, local_payload_velocity);
			const Vec2D IntendedVelocity_fromWheelAngle = GlobalToLocal(IntendedDirection, IntendedVelocities);
			const double Skid_Velocity = aligned_to_Intended_velocity.x();  //velocity to apply friction force to can be in either direction
			m_Friction.SetVelocity(Skid_Velocity);
			const double friction_x = m_Friction.GetFrictionalForce(dTime_s, FrictionMode);
			//printf("|fnx=%.2f|",friction_x);
			double friction_y = 0.0;
			//Test for friction Y (only when controls for Y are idle)
			if (IsZero(IntendedVelocity_fromWheelAngle.y(),0.01))
			{
				double combined_velocity_magnitude=0.0;
				for (size_t i = 0; i < 4; i++)
					combined_velocity_magnitude += m_CurrentVelocities_callback().Velocity.AsArray[i];
				//while we are here... if we are stopped then we need to do the same for the y Component
				if (IsZero(combined_velocity_magnitude,0.01))
				{
					m_Friction.SetVelocity(aligned_to_Intended_velocity.y());
					friction_y = m_Friction.GetFrictionalForce(dTime_s, FrictionMode);
				}
			}
			//now to get friction force to be applied to x component
			const Vec2D global_friction_force(friction_x, friction_y);
			const Vec2D local_friction_force = LocalToGlobal(IntendedDirection, global_friction_force);
			//We can now consume these forces into the payload
			m_Payload.ApplyFractionalForce(local_friction_force, dTime_s);
			//Same goes for the angular velocity as this should not be sliding around
			if (IsZero(m_Input.GetAngularVelocity(), 0.01)&&(m_Payload.GetAngularVelocity()!=0.00))
			{
				const double AngularVelocity = m_Payload.GetAngularVelocity();
				//Avoid small fractions by putting a bite in the threshold
				if (fabs(AngularVelocity) < 0.0001)
				{
					m_Payload.SetAngularVelocity(0.0);
				}
				else
				{
					//The way to think of this is that in space something could spin forever and it is friction that would stop it
					//lack of friction (like a top or something on bearings demonstrates this as well) once the robot spins it is
					//the friction of the wheels that stops the spin.  To be compatible to our units of force (which work with KMS i.e. meters)
					//we convert our angular velocity into linear compute this force and scale back down
					//force = mass * acceleration, acceleration = vel_a-vel_b, vel = distance / time
					//proof... using 1 for time distance a is 3 and distance b is 2.    3-2 (meters) a =1 meters per second square
					//for radians, for now let's assume Pi * D= 2.   1/D(3-2)= 1/D  so we can scale this back down to radians, where D is the turning diameter
					const double turning_diameter = m_Input.GetDriveProperties().GetTurningDiameter();
					const double to_linear = Pi * turning_diameter;
					const double rot_linear = AngularVelocity * to_linear;
					assert(turning_diameter != 0.0);  //sanity check
					m_Friction.SetVelocity(rot_linear);
					const double friction_rot = m_Friction.GetFrictionalForce(dTime_s, FrictionMode) / to_linear; //as radians
					m_Payload.ApplyFractionalTorque(friction_rot, dTime_s);
				}
			}
		}
		// we could m_Payload.TimeChangeUpdate() at some point
		const Vec2D current_payload_velocity = m_Payload.GetLinearVelocity();
		//printf("--%.2f,", current_payload_velocity.y());
		const double current_payload_angular_velocity = m_Payload.GetAngularVelocity();
		const SwerveVelocities &AdjustedToExisting=m_Output.UpdateFromExisting(current_payload_velocity.y(), current_payload_velocity.x(), 
			current_payload_angular_velocity, InitialVelocitiesForPayload);
		//Now we can apply the torque to the wheel
		for (size_t i = 0; i < 4; i++)
		{
			Potentiometer_Tester4& encoder = m_Encoders[i];
			Framework::Base::PhysicsEntity_1D& wheel_model = encoder.GetWheelModel_rw();
			//const double Force = m_Output.GetIntendedVelocitiesFromIndex(i);
			//const double Torque = Force * Inches2Meters(m_WheelDiameter_In * 0.5);
			const double WheelVelocity_fromPayload = AdjustedToExisting.Velocity.AsArray[i] / Inches2Meters(m_WheelDiameter_In * 0.5);
			#if 0
			//To set the velocity directly it works with motor velocity so we'll need to apply the inverse gear ratio
			wheel_model.SetVelocity(WheelVelocity_fromPayload * (1.0/encoder.GetGearReduction()));
			#else
			//Instead of setting the velocity directly, we can apply as a torque, this way if we skid we can apply less force for adjustments
			//Note: In simulation we work with motor velocity, and do not care about where the physical encoder is, because the actual encoder
			//output is linear velocity.  Outside the simulation using a real encoder will need to use the encoder's reduction to have the 
			//same output of linear velocity
			const double motor_velocity = WheelVelocity_fromPayload * (1.0 / encoder.GetGearReduction());
			double torque_load = (motor_velocity - wheel_model.GetVelocity()) / dTime_s * wheel_model.GetMomentofInertia();
			//separating this out for easy of debugging:
			if (IsSkidding)
				torque_load = 0.0; //This is not correct, but will help us visualize the skid this is some impact on the velocity though
			wheel_model.ApplyFractionalTorque(torque_load, dTime_s);

			#endif
			//Now that the velocity has taken effect we can add in the adverse torque
			const double WheelVelocity= wheel_model.GetVelocity() * encoder.GetGearReduction();
			const double LinearVelocity = WheelVelocity * Inches2Meters(m_WheelDiameter_In * 0.5);
			//finally update our odometry, note we work in linear velocity so we multiply by the wheel radius
			m_CurrentVelocities_callback().Velocity.AsArray[i] = LinearVelocity;
		}
	}
	void SetCurrentVelocities_Callback(std::function<SwerveVelocities&()> callback)
	{
		m_CurrentVelocities_callback=callback;
	}
	void ResetPos()
	{
		for (size_t i = 0; i < 4; i++)
			m_Encoders[i].ResetPos();
		m_Payload.ResetVectors();
	}
	const Framework::Base::PhysicsEntity_2D& GetPayload() const
	{
		//read access for advanced sensors to use
		return m_Payload;
	}
};

#pragma endregion


#pragma region _Simulated Odometry Internal_
class SimulatedOdometry_Internal
{
private:
	static inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}

	#pragma region _member vars_
	SwerveVelocities m_CurrentVelocities;
	std::function<SwerveVelocities()> m_VoltageCallback;
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	double m_current_position[4] = {};  //keep track of the pot's position of each angle
	//const Framework::Base::asset_manager *m_properties=nullptr;  <----reserved
	struct properties
	{
		double swivel_max_speed[4];
	} m_bypass_properties;
	Potentiometer_Tester4 m_Potentiometers[4];
	SwerveEncoders_Simulator4 m_EncoderSim4;

	class AdvancedSensors
	{
	private:
		SimulatedOdometry_Internal* m_pParent;
		Vec2D m_current_position;
		double m_current_heading = 0.0;
	public:
		AdvancedSensors(SimulatedOdometry_Internal* parent) : m_pParent(parent)
		{
		}
		void TimeSlice(double d_time_s)
		{
			using namespace Framework::Base;
			//grab our velocities
			const Framework::Base::PhysicsEntity_2D& payload = m_pParent->m_EncoderSim4.GetPayload();
			const double AngularVelocity = payload.GetAngularVelocity();
			//update our heading
			m_current_heading += AngularVelocity * d_time_s;
			m_current_heading=NormalizeRotation2(m_current_heading);
			//With latest heading we can adjust our position delta to proper heading
			const Vec2D local_velocity = payload.GetLinearVelocity();
			const Vec2D global_velocity = LocalToGlobal(m_current_heading, local_velocity);
			//Now we can advance with the global
			m_current_position += global_velocity * d_time_s;
		}
		void Reset()
		{
			m_current_position = Vec2D(0.0, 0.0);
			m_current_heading = 0.0;
		}
		Vec2D Vision_GetCurrentPosition() const
		{
			return m_current_position;
		}
		double GyroMag_GetCurrentHeading() const
		{
			return m_current_heading;
		}
	} m_AdvancedSensors=this;

	bool m_UseBypass = true;
	#pragma endregion
	void SetHooks(bool enabled)
	{
		if (enabled)
		{
			m_EncoderSim4.SetCurrentVelocities_Callback(
				[&]() -> SwerveVelocities &
			{
				return m_CurrentVelocities;
			}
			);
		}
		else
			m_EncoderSim4.SetCurrentVelocities_Callback(nullptr);
	}
public:
	void Init(const Framework::Base::asset_manager *props)
	{
		using namespace ::properties::registry_v1;
		//m_properties = props;  <---- reserved... most likely should restrict properties here
		m_UseBypass = props ? props->get_bool(csz_Build_bypass_simulation, m_UseBypass) : true;
		//Since we define these props as we need them the client code will not need default ones
		m_bypass_properties.swivel_max_speed[0] =
			m_bypass_properties.swivel_max_speed[1] =
			m_bypass_properties.swivel_max_speed[2] =
			m_bypass_properties.swivel_max_speed[3] = 8.0;

		//If we are using bypass, we are finished
		if (m_UseBypass)
			return;
		m_EncoderSim4.Initialize(props);
		for (size_t i = 0; i < 4; i++)
			m_Potentiometers[i].Initialize(i, props);

		ResetPos();
	}
	void ResetPos()
	{
		if (m_UseBypass)
		{
			m_current_position[0]= m_current_position[1] = m_current_position[2] = m_current_position[3] = 0.0;
		}
		else
		{
			for (size_t i = 0; i < 4; i++)
				m_Potentiometers[i].ResetPos();
			m_EncoderSim4.ResetPos();
			m_AdvancedSensors.Reset();
		}
	}
	void ShutDown()
	{
		SetHooks(false);
	}
	SimulatedOdometry_Internal()
	{
		SetHooks(true);
	}
	~SimulatedOdometry_Internal()
	{
		ShutDown();
	}
	void SetVoltageCallback(std::function<SwerveVelocities()> callback)
	{
		//Input get it from client
		m_VoltageCallback = callback;
		m_EncoderSim4.SetVoltageCallback(callback);
	}
	//Run the simulation time-slice
	void TimeSlice(double d_time_s)
	{
		if (m_UseBypass)
		{
			//If only life were this simple, but alas robots are not god-ships
			m_CurrentVelocities = m_VoltageCallback();
			//convert voltages back to velocities
			//for drive this is easy
			for (size_t i = 0; i < 4; i++)
			{
				m_CurrentVelocities.Velocity.AsArray[i] *= m_maxspeed;
			}
			//for position we have to track this ourselves
			for (size_t i = 0; i < 4; i++)
			{
				//go ahead and apply the voltage to the position... this is over-simplified but effective for a bypass
				//from the voltage determine the velocity delta
				const double velocity_delta = m_CurrentVelocities.Velocity.AsArray[i + 4] * m_bypass_properties.swivel_max_speed[i];
				m_current_position[i] = NormalizeRotation2(m_current_position[i] + velocity_delta * d_time_s);
				m_CurrentVelocities.Velocity.AsArray[i + 4] = m_current_position[i];
			}
		}
		else
		{
			SwerveVelocities CurrentVoltage;
			CurrentVoltage = m_VoltageCallback();
			for (size_t i = 0; i < 4; i++)
			{
				//We'll put the pot update first in case the simulation factors in the direction (probably shouldn't matter though)
				m_Potentiometers[i].SetTimeDelta(d_time_s);
				m_Potentiometers[i].UpdatePotentiometerVoltage(CurrentVoltage.Velocity.AsArray[i + 4]);
				m_Potentiometers[i].TimeChange();
				//cannot normalize here
				//m_current_position[i] = NormalizeRotation2(m_Potentiometers[i].GetPotentiometerCurrentPosition());
				m_current_position[i] = m_Potentiometers[i].GetPotentiometerCurrentPosition();
				m_CurrentVelocities.Velocity.AsArray[i + 4] = m_current_position[i];
			}
			//Ensure potentiometers are updated before calling sim4
			m_EncoderSim4.TimeChange(d_time_s);
			m_AdvancedSensors.TimeSlice(d_time_s);
		}
	}
	const SwerveVelocities &GetCurrentVelocities() const
	{
		//Output: contains the current speeds and positions of any given moment of time
		return m_CurrentVelocities;
	}
	#pragma region _advanced_sensor_methods_
	bool Sim_SupportVision() const
	{
		return true;
	}
	Vec2D Vision_GetCurrentPosition() const
	{
		return m_AdvancedSensors.Vision_GetCurrentPosition();
	}
	bool Sim_SupportHeading() const
	{
		return true;
	}
	double GyroMag_GetCurrentHeading() const
	{
		return m_AdvancedSensors.GyroMag_GetCurrentHeading();
	}
	#pragma endregion
};
#pragma endregion
#pragma region _wrapper methods_
SimulatedOdometry::SimulatedOdometry()
{
	m_simulator = std::make_shared<SimulatedOdometry_Internal>();
}
void SimulatedOdometry::Init(const Framework::Base::asset_manager* props)
{
	m_simulator->Init(props);
}
void SimulatedOdometry::ResetPos()
{
	m_simulator->ResetPos();
}
void SimulatedOdometry::Shutdown()
{
	m_simulator->ShutDown();
}
void SimulatedOdometry::SetVoltageCallback(std::function<SwerveVelocities()> callback)
{
	m_simulator->SetVoltageCallback(callback);
}
void SimulatedOdometry::TimeSlice(double d_time_s)
{
	m_simulator->TimeSlice(d_time_s);
}
const SwerveVelocities &SimulatedOdometry::GetCurrentVelocities() const
{
	return m_simulator->GetCurrentVelocities();
}
bool SimulatedOdometry::Sim_SupportVision() const
{
	return m_simulator->Sim_SupportVision();
}
Vec2D SimulatedOdometry::Vision_GetCurrentPosition() const
{
	return m_simulator->Vision_GetCurrentPosition();
}
bool SimulatedOdometry::Sim_SupportHeading() const
{
	return m_simulator->Sim_SupportHeading();
}
double SimulatedOdometry::GyroMag_GetCurrentHeading() const
{
	return m_simulator->GyroMag_GetCurrentHeading();
}
#pragma endregion
}}