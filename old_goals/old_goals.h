class Goal_ElevatorControl : public AtomicGoal
{
  public:
    Goal_ElevatorControl(ActiveCollection* activeCollection, double target) : m_activeCollection(activeCollection) , m_target(target)
    {
      #ifndef _Win32
      m_pot = (PotentiometerItem*)activeCollection->Get("pot");
      #endif
    }
    virtual void Activate();
    virtual Goal::Goal_Status Process(double dTime);
    virtual void Terminate();
  private:
    ActiveCollection* m_activeCollection;
    PotentiometerItem* m_pot;
    double m_target;

    double m_currentPos;
    double error, errorPrior = 0, integ, deriv;
    const double bias = 0, kp = 4.5, ki = 0, kd = .003;
    const double FREEDOM = 0.02;
    const double MAX_POWER = .6;
};

class Goal_RelativeElevatorControl : public AtomicGoal
{
    public:
      Goal_RelativeElevatorControl(ActiveCollection* activeCollection, double delta)
      {
          double target = ((PotentiometerItem*)activeCollection->Get("pot"))->Get() + delta;
          goal = new Goal_ElevatorControl(activeCollection, target);
          m_activeColelction = activeCollection;
      }
      virtual void Activate();
      virtual Goal::Goal_Status Process(double dTime);
      virtual void Terminate();
    private:
      Goal_ElevatorControl* goal;
      ActiveCollection* m_activeColelction;
};

class Goal_Turn : public Goal_Wait_ac
{
public:
  Goal_Turn(ActiveCollection *activeCollection, double angle, double timeOut = 3.0) : Goal_Wait_ac(activeCollection, timeOut) 
  {
    m_navx = activeCollection->GetNavX();
  }
  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

private:
  double m_target;
  double m_power;
  const double m_freedom = 1; //code can be within +/- n degrees of target
  NavX *m_navx;

  double kp = 1, ki = 0, kd = .05;
  double bias = 0; //bias is a constant that is added to the output. We most likely will not use it but it is here if needed.
  double propError, integ, deriv;
  double errorPrior;
};

/* Goal_DriveStraight
    * This goal uses encoder to drive a specified amount forward and navx to stay straight
    */
class Goal_DriveStraight : public Goal_Wait_ac
{
public:
  //*standard constructor
  Goal_DriveStraight(ActiveCollection *activeColelction, double dist, double power, double timeOut) : Goal_Wait_ac(activeColelction, timeOut)
  {
    m_distTarget = dist;
    m_encLeft = m_activeCollection->GetEncoder("enc0");
    m_encRight = m_activeCollection->GetEncoder("enc1");
    m_leftPower = m_rightPower = power;
  }
  //*optional constructor without timeOut, calls standard with default timeOut value
  Goal_DriveStraight(ActiveCollection *activeCollection, double dist, double power) : Goal_DriveStraight(activeCollection, dist, power, 7.0) {}
  //*constructor that takes a distance object as a parameter
  Goal_DriveStraight(ActiveCollection *activeCollection, Distance *dist, double power, double timeOut) : Goal_DriveStraight(activeCollection, dist->inATick(), power, timeOut) {}
  //*constructor that takes distance object and uses default timeOut value
  Goal_DriveStraight(ActiveCollection *activeCollection, Distance *dist, double power) : Goal_DriveStraight(activeCollection, dist, power, 7.0) {}

  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

private:
  double m_distTarget;
  double m_distTraveled = 0;
  double m_leftPower, m_rightPower;
  EncoderItem *m_encLeft, *m_encRight;

  double kp = 0, ki = 0, kd = 0;
  double bias = 0; //bias is a constant that is added to the output. We most likely will not use it but it is here if needed.
  double error, integ, deriv;
  double errorPrior;
};

//TODO vision aligment:
/* if x is out of bounds, turn towards target
 * then, if radius (distance) out of bounds, drive straight towards target 
 * SetDrive() not turn goal
 */
class Goal_VisionAlign : public Goal_TimeOut
{
public:
  Goal_VisionAlign(ActiveCollection *activeCollection, VisionTarget *target, double timeOut = 7.0) : Goal_TimeOut(activeCollection, timeOut)
  {
    m_inst = nt::NetworkTableInstance::GetDefault(); //!Network tables
    m_visionTable = m_inst.GetTable("VISION_2019");  //!Vision network table
    m_target = target;
    m_currentTarget = new VisionTarget();
  }

  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

private:
  void updateVision()
  {
    m_currentTarget->setX(m_visionTable->GetNumber("X", 0));
    m_currentTarget->setY(m_visionTable->GetNumber("Y", 0));
    m_currentTarget->setRadius(m_visionTable->GetNumber("RADIUS", 0));
    //Area = m_visionTable->GetNumber("AREA", 0);
    Height = m_visionTable->GetNumber("HEIGHT", 0);
    Width = m_visionTable->GetNumber("WIDTH", 0);
    HasTarget = m_visionTable->GetBoolean("HASTARGET", false);
  }
  int Height, Width;
  bool HasTarget;

  //constants for motor speeds. These are multiplied by vision error
  const double TURN_KP = 0.0025;   //?guess for right now
  const double STRAIGHT_KP = 0.05; //?guess for right now

  nt::NetworkTableInstance m_inst;        //!Network tables
  shared_ptr<NetworkTable> m_visionTable; //!Vision table
  VisionTarget *m_target;
  VisionTarget *m_currentTarget;
};

// /* Goal_ElevatorPosition
//  * Positions: 0, 1, 2, 3
//  * 0 = all the way down, 3 = all the way up
//  * Offset: small increase in height for cargo intake on rocket
//  */
// class Goal_ElevatorPosition
// {
//   public:

// };
#pragma endregion

#pragma region UtilGoals

/* Goal_Hatch
    This goal is meant to manipulate the mechanisms that would result in outtaking the hatch (this will need adjusting)
    */
class Goal_Hatch : public Goal_Wait_ac
{
public:
  Goal_Hatch(ActiveCollection *activeCollection, double timeOut) : Goal_Wait_ac(activeCollection, 1)
  {
    m_Status = eActive;
    m_timeOut = timeOut;
    m_currentTime = 0; ///////////////////////////////////////////////////////////////////////////////////////////////////
  }
  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

protected:
  ActiveCollection *m_activeCollection;
};

/* Goal_OneHatch
    This goal is meant to score one hatch on the cargo during autonomous
    */
class Goal_OneHatchFrontShip : public CompositeGoal
{
public:
  Goal_OneHatchFrontShip(ActiveCollection *activeCollection, string position = "none")
  {
    m_activeCollection = activeCollection;
    m_position = position;
    m_Status = eInactive;
  }

  virtual void Activate();

private:
  string m_position;
  ActiveCollection* m_activeCollection;
};