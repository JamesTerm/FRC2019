#pragma once


#ifdef Robot_TesterCode
class Actor_Text;

class COMMON_API EntityPropertiesInterface
{
	public:
		virtual const Vec2D &GetPos_m() const =0;
		virtual double GetAtt_r() const=0;
		virtual const std::string &GetName() const=0;
		virtual const Vec2D &GetDimensions() const=0;
		virtual const double &GetIntendedOrientation() const=0;
		//perform an optional custom update on the ship
		virtual void UI_Init(Actor_Text *parent) {}
		virtual void custom_update(osg::NodeVisitor *nv, osg::Drawable *draw,const osg::Vec3 &parent_pos) {}
		//This is called when it is detected the size has changed
		virtual void Text_SizeToUse(double SizeToUse) {}
		//AddOrRemove if true it is add false is remove
		virtual void UpdateScene (osg::Geode *geode, bool AddOrRemove) {}
};
#else
class COMMON_API EntityPropertiesInterface
{
};

inline void NormalizeRotation(double &Rotation)
{
	const double Pi2=M_PI*2.0;
	//Normalize the rotation
	if (Rotation>M_PI)
		Rotation-=Pi2;
	else if (Rotation<-M_PI)
		Rotation+=Pi2;
}

#endif

class COMMON_API Entity1D
{
	private:
		friend class Entity1D_Properties;

		Base::EventMap* m_eventMap;
		double m_StartingPosition;  //the position used when reset position is called
		double m_Dimension;
		double m_Position;
		std::string m_Name;
		bool m_BypassPos_Update;  //used to preserve pos during a ResetPos() call
	public:
		Entity1D(const char EntityName[]);

		//This allows the game client to setup the ship's characteristics
		virtual void Initialize(Base::EventMap& em, const Entity1D_Properties *props=NULL);
		virtual ~Entity1D(); //Game Client will be nuking this pointer
		const std::string &GetName() const {return m_Name;}
		virtual void TimeChange(double dTime_s);
		PhysicsEntity_1D &GetPhysics() {return m_Physics;}
		const PhysicsEntity_1D &GetPhysics() const {return m_Physics;}
		virtual double GetDimension() const {return m_Dimension;}
		virtual double GetStartingPosition() const {return m_StartingPosition;}
		virtual void ResetPosition(double Position);
		virtual void ResetPos();
		// This is where both the entity and camera need to align to, by default we use the actual position
		virtual const double &GetIntendedPosition() const {return m_Position;}
		Base::EventMap* GetEventMap(){return m_eventMap;}

		virtual double GetPos_m() const {return m_Position;}
		//This is used when a sensor need to correct for the actual position
		void SetPos_m(double value) {m_Position=value;}
		//Be sure to always set this back to false!
		void SetBypassPos_Update(bool bypass) {m_BypassPos_Update=bypass;}
		bool GetBypassPos_Update() const {return m_BypassPos_Update;}
	protected: 
		///This gives derived class the ability to manipulate the displacement
		/// \ret true if this is to be used and manipulated, false uses the default displacement
		virtual bool InjectDisplacement(double DeltaTime_s,double &PositionDisplacement) {return false;}

		PhysicsEntity_1D m_Physics;
		bool m_IsAngular;
};


class Ship_Tester;
//This contains everything the AI needs for game play; Keeping this encapsulated will help keep a clear division
//of what Entity3D looked like before applying AI with goals
#ifdef Robot_TesterCode
typedef Entity2D Entity2D_Kind;
#else
namespace Entity2D_Kind=Framework::Base;
#endif

//Note Entity2D should not know anything about an actor
class COMMON_API Entity2D : public EntityPropertiesInterface
{
	public:
		#ifdef Robot_TesterCode
		class EventMap : public GG_Framework::UI::EventMap
		{
		public:
			EventMap(bool listOwned = false) : GG_Framework::UI::EventMap(listOwned) {}
		};
		#endif
	private:
		friend class Ship_Tester;
		friend class Entity_Properties;
		struct PosAtt
		{
			Vec2D m_pos_m;
	
			//2d Orientation:
			double m_att_r;  //a.k.a heading
			//measurement in radians where 0 points north, pi/2 = east,  pi=south, and -pi/2 (or pi + pi/2) = west
			//from this a normalized vector can easily be computed by
			//x=sin(heading) y=cos(heading)
			//We can keep the general dimensions of the entity
			//Note: we do not need pitch or roll axis so this keeps things much simpler
		} m_PosAtt_Buffers[2];
	
		//All read cases use the read pointer, all write cases use the write pointer followed by an interlocked exchange of the pointers
		PosAtt *m_PosAtt_Read,*m_PosAtt_Write; 
		double m_att_r;  //I need a dedicated heading for physics to use
		void UpdatePosAtt();

		Vec2D m_DefaultPos;
		double m_DefaultAtt;
		Entity2D_Kind::EventMap* m_eventMap;
	
		Vec2D m_Dimensions;
		std::string m_Name;
		bool m_BypassPosAtt_Update;  //used to preserve pos att during a ResetPos() call

	public:
		Entity2D(const char EntityName[]);

		//This allows the game client to setup the ship's characteristics
		virtual void Initialize(Entity2D_Kind::EventMap& em, const Entity_Properties *props=NULL);
		virtual ~Entity2D(); //Game Client will be nuking this pointer
		const std::string &GetName() const {return m_Name;}
		virtual void TimeChange(double dTime_s);
		FlightDynamics_2D &GetPhysics() {return m_Physics;}
		const PhysicsEntity_2D &GetPhysics() const {return m_Physics;}
		virtual const Vec2D &GetDimensions() const {return m_Dimensions;}
		virtual void ResetPos();
		// This is where both the vehicle entity and camera need to align to, by default we use the actual orientation
		virtual const double &GetIntendedOrientation() const {return m_PosAtt_Read->m_att_r;}
		Entity2D_Kind::EventMap* GetEventMap(){return m_eventMap;}

		//from EntityPropertiesInterface
		virtual const Vec2D &GetPos_m() const {return m_PosAtt_Read->m_pos_m;}
		virtual double GetAtt_r() const {return m_PosAtt_Read->m_att_r;}

		//This is the position used when ResetPos() is called
		void SetDefaultPosition(const Vec2D &pos) {m_DefaultPos=pos;}
		void SetDefaultAttitude(double att) {m_DefaultAtt=att;}
		//Be sure to always set this back to false!
		void SetBypassPosAtt_Update(bool bypass) {m_BypassPosAtt_Update=bypass;}
		bool GetBypassPosAtt_Update() const {return m_BypassPosAtt_Update;}
	protected: 
		FlightDynamics_2D m_Physics;
		///This gives derived class the ability to manipulate the displacement
		/// \ret true if this is to be used and manipulated, false uses the default displacement
		virtual bool InjectDisplacement(double DeltaTime_s,Vec2D &PositionDisplacement,double &RotationDisplacement) {return false;}
};

typedef Entity2D Ship;
