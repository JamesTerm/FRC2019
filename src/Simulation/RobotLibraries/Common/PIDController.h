#pragma once
#include <queue>
#include "../Base/Misc.h"

//To assist in better control on the PID in addition to the linearization of the victor, there can also be latency from the time the voltage is
//applied to the time it takes effect.  This class makes it easy for systems to account for the latency to reduce error for PID and possibly
//unnecessary oscillation that would otherwise occur
class LatencyFilter
{
	public:
		LatencyFilter(double Latency=0.0);
		/// \param input is the actual position where it is
		/// \param dTime_s is the slice of time for this call
		/// \ret Tries to return the actual position of where it was m_Latency_ms ago; otherwise will return a more current position
		double operator()(double input,double dTime_s);
		double operator()();  //This is a passive operation that simply allows multiple calls to obtain the last known value
		void SetLatency(double Latency);
	private:
		std::queue<double> m_Queue; //This grows as needed
		double m_Latency_s;  //Latency in seconds
};

//This is the same idea and interface to the latency filter, but instead of inducing latency it will predict what the value will be by first
//derivative prediction
class LatencyPredictionFilter
{
	public:
		LatencyPredictionFilter(double Latency=0.0);
		/// \param input is the actual position where it is
		/// \target_point is the current predicted point from the force feed 
		/// \param dTime_s is the slice of time for this call
		/// \ret Tries to return the actual position of where it was m_Latency_ms ago; otherwise will return a more current position
		double operator()(double input,double target_point,double dTime_s);
		//This version does not blend in the target rate
		/// \param input is the actual position where it is
		/// \param dTime_s is the slice of time for this call
		/// \ret Tries to return the actual position of where it was m_Latency_ms ago; otherwise will return a more current position
		double operator()(double input,double dTime_s);
		double operator()();  //This is a passive operation that simply allows multiple calls to obtain the last known value
		void SetLatency(double Latency);
	private:
		double m_Predicted;
		//cache two iterations of values for prediction
		double m_Prev_Input,m_Prev_Target;
		double m_Latency_s;  //Latency in seconds
};

class COMMON_API KalmanFilter
{
	public:
		KalmanFilter();
		/// \return the filtered value
		double operator()(double input);
		void Reset();
	private:
	    //initial values for the Kalman filter
	    double m_x_est_last;
	    double m_last;
	    //the noise in the system
	    const double m_Q;
	    const double m_R;
	    bool m_FirstRun; //This avoids a stall when first starting
};

/// This manages a PID control loop.  This was originally written for First WPI library, but refactored to be non-threaded, where both input and output
/// can be called on within this class.  It manages the integral calculations, and provides the PIDOutput
class COMMON_API PIDController2
{
	public:
		PIDController2(
			double p,					///< proportional coefficient
			double i,					///< integral coefficient
			double d,					///< derivative coefficient
			bool AutoResetI=true,		///< If both setpoint and input are zero total error is reset (great for all velocity control cases except for lifting mechanisms)
			double maximumOutput=1.0,	///< maximum output
			double minimumOutput=-1.0,	///< minimum output|
			double maximumInput=1.0,	///< maximum input - limit setpoint to this
			double minimumInput=-1.0,	///< minimum input - limit setpoint to this
			double m_tolerance=.05,		///<the percentage error that is considered on target
			bool continuous=false,		///< do the endpoints wrap around? eg. Absolute encoder
			bool enabled=false	 		///< If client knows all the above, set to true; otherwise enable use Enable() for late binding
			);
		~PIDController2();

		///This is the main method which performs the computations, and must be called for each time slice
		/// \return the final output result
		/// \param setpoint is the actual desired position.  \note The range of this must be managed by client code!
		/// \param input is the actual position where it is
		/// \param dTime_s is the slice of time for this call
		double operator()(double setpoint,double input,double dTime_s);

		///	This is always centered on zero and constrained the the max and min outs
		///	\return the latest calculated output
		double Get();

		///Set the PID controller to consider the input to be continuous, rather than using the max and min in as constraints, it considers them to
		///be the same point and automatically calculates the shortest route to the setpoint.
		///	\param continuous Set to true turns on continuous, false turns off continuous
		void SetContinuous(bool continuous = true);
		void SetAutoResetI(bool AutoReset=true);

		void SetInputRange(double minimumInput, double maximumInput);
		void SetOutputRange(double mimimumOutput, double maximumOutput);
		void SetPID(double p, double i, double d);
		double GetP();
		double GetI();
		double GetD();

		double GetError();  ///< Returns the current difference of the input from the setpoint
		double GetTotalError() {return m_totalError;}  //Help track I

		///Set the percentage error which is considered tolerable for use with OnTarget.
		///	\param percent percentage of error which is tolerable
		void SetTolerance(double percent);

		/// \return true if the error is within the percentage of the total input range,
		/// determined by SetTolerance. 
		/// \note this uses the input range therefore ensure it has been set properly
		bool OnTarget();
		
		void Enable();
		void Disable();

		///Resets the previous error, the integral term  (This does not disable as it does in the WPI lib)
		void Reset();
		/// A quick call often function that zero's the I only
		void ResetI();
		void ResetI(double totalError);  //allow client to set to specific value
	private:
		double m_P;			// factor for "proportional" control
		double m_I;			// factor for "integral" control
		double m_D;			// factor for "derivative" control
		double m_maximumOutput;	// |maximum output|
		double m_minimumOutput;	// |minimum output|
		double m_maximumInput;		// maximum input - limit setpoint to this
		double m_minimumInput;		// minimum input - limit setpoint to this
		double m_prevError;	// the prior sensor input (used to compute velocity)
		double m_totalError; //the sum of the errors for use in the integral calculation
		double m_tolerance;	//the percentage error that is considered on target
		double m_error;
		double m_result;
		bool m_continuous;	// do the endpoints wrap around? eg. Absolute encoder
		bool m_enabled;		//is the pid controller enabled
		bool m_AutoResetI;
};
