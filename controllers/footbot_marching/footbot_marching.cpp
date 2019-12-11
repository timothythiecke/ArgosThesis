/* Include the controller definition */
#include "footbot_marching.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>
#include <argos3/core/simulator/simulator.h>


/****************************************/
/****************************************/

CFootBotMarching::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}

void CFootBotMarching::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "spontaneous_switching_prob", SpontSwitchingProb);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void CFootBotMarching::SStateData::Reset() {
   State = CLOCKWISE;
}

bool CFootBotMarching::SStateData::IsClockwise() {
   if(State == CLOCKWISE){return true;}
   return false;
}

/****************************************/
/****************************************/

CFootBotMarching::SMotionParams::SMotionParams() :
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(1.0f),
   m_fWheelDefaultVelocity(1.0f),
   velFactor(1.0f),
   b_isStopped(false),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)){}
                           
void CFootBotMarching::SMotionParams::Init(TConfigurationNode& t_node) {
   try {
	   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
	   GetNodeAttributeOrDefault(t_node, "velocity", velFactor, velFactor);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller motion parameters.", ex);
   }
}

void CFootBotMarching::SMotionParams::SaveWheelsVelocity() {
	m_fWheelDefaultVelocity = m_fWheelVelocity;
}

void CFootBotMarching::SMotionParams::SetWheelsVelocity(Real f_velocity) {
	SaveWheelsVelocity();
	m_fWheelVelocity = f_velocity;
}

void CFootBotMarching::SMotionParams::ReSetWheelsVelocity() {
	m_fWheelVelocity = m_fWheelDefaultVelocity;
	b_isStopped = false;
}

/****************************************/
/****************************************/

CFootBotMarching::CFootBotMarching() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void CFootBotMarching::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_marching><actuators> and
    * <controllers><footbot_marching><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
      /*
       * Parse XML parameters
       */
      /* Motion algorithm */
      m_sMotionParams.Init(GetNode(t_node, "motion"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */

   	// Do the same for the range parameters
   	TConfigurationNode& tRange = GetNode(t_node, "range");
    GetNodeAttribute(tRange, "lower_bound", mRangeLowerBound);
	if (mRangeLowerBound < 0.0)
	{
		LOG << "Range lower bound is less than zero, this may lead to undefined behaviour!";
	}

    GetNodeAttribute(tRange, "upper_bound", mRangeUpperBound);
	if (mRangeUpperBound < 0.0)
	{
		mRangeUpperBound = std::numeric_limits<Real>::max();
	}

    GetNodeAttribute(tRange, "step", mRangeStep);

	bool state = true;
    GetNodeAttribute(tRange, "probabilistic", state);
	// ^ Note: Argos will stop working if it can't parse a node attribute, so we should move this to a try catch block
	// THROW_ARGOSEXCEPTION_NESTED("Error with initializing s.", ex);
	if (state)
	{
		mRangeDecisionState = CFootBotMarching::ERangeDecisionMakingState::Probabilistic;
	}
	else
	{
		mRangeDecisionState = CFootBotMarching::ERangeDecisionMakingState::Binary;
	}

	GetNodeAttribute(tRange, "symmetric", mRangeSymmetric);

	mHistoryData.reserve(200);

   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}

/****************************************/
/****************************************/

void CFootBotMarching::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Clear up the last state info */
   m_pcRABA->ClearData();
   /* Set LED color */
   m_pcLEDs->SetAllColors(CColor::RED);
   m_sMotionParams.ReSetWheelsVelocity();
   
   m_unID = (int) std::stoi(m_strId.substr (2,5),nullptr,10); // The Id integer number that is sent by the actuator, converted from strID
   
   cDegGoal.SetValue(90);
   b_rabChanged = false;
   b_breakdown = false;
   
   timer=0;
   newRABRange = 0.0;
   
   f_fracLeft = 0.0;
   f_fracRight = 0.0;
   f_fracLeft_Old = 0.0;
   f_fracRight_Old = 0.0;
   f_fracLeft_Change = 0.0;
   f_fracRight_Change = 0.0;
   f_fracLeft_AvgChange = 0.0;
   f_fracRight_AvgChange = 0.0;
   variance_of_change_left = 0.0;
   variance_of_change_right = 0.0;
   
   if(m_pcRNG->Uniform(m_sStateData.ProbRange) < 0.5){
			m_pcLEDs->SetAllColors(CColor::GREEN);
			m_sMotionParams.SetWheelsVelocity(m_sMotionParams.m_fWheelVelocity*(-1.0));
   }
}

/****************************************/
/****************************************/

void CFootBotMarching::SetWheelSpeedsFromVector() {
   /* Get the heading angle */
   const CVector2 c_heading = CVector2::X;
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = fHeadingLength;
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   CDegrees cAngle;
   cAngle.SetValue(1.3);

	/* Both wheels go straight, but one is faster than the other */
    Real fSpeedFactor = (ToRadians(cAngle) - Abs(cHeadingAngle)) / ToRadians(cAngle) ;
    fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
    fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);

   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

Real CFootBotMarching::GetNewRABRange(){
	return newRABRange;
}

/****************************************/
/****************************************/

bool CFootBotMarching::IsRABChanged(){
	return b_rabChanged;
}

/****************************************/
/****************************************/

void CFootBotMarching::SetNewRABRange(Real newRange){
	newRABRange = newRange;
	b_rabChanged = true;
}

/****************************************/
/****************************************/

Real CFootBotMarching::GetVelocity(){
	return m_sMotionParams.m_fWheelVelocity;
}

/****************************************/
/****************************************/

void CFootBotMarching::SwitchState(){
	m_sMotionParams.SetWheelsVelocity(-m_sMotionParams.m_fWheelVelocity);
}

/****************************************/
/****************************************/

CCI_RangeAndBearingSensor::TReadings CFootBotMarching::GetTPackets(){
	//LOG << "here" << std::endl;
   return m_pcRABS->GetReadings();
}

/****************************************/
/****************************************/

void CFootBotMarching::SetFakeTPackets(CCI_RangeAndBearingSensor::TReadings& fakeTPackets)
{
	if (mIsNodeOfInterest)
	{
		LOG << "Deg: " << degree << " TPackets size: " << tPackets.size() << std::endl;
	}

	degree = fakeTPackets.size();
	tPackets = m_pcRABS->GetReadings();
	
	if (mIsNodeOfInterest)
	{
		LOG << "Deg: " << degree << " TPackets size: " << tPackets.size() << std::endl;
	}

	/*tPackets.clear();
	tPackets = std::move(fakeTPackets);
	/*for(size_t i = 0; i < degree; i++)
	{
		tPackets.push_back(fakeTPackets[i]);
	}*/	
}

/****************************************/
/****************************************/

bool CFootBotMarching::GetBreakdown(){
   return b_breakdown;
}

/****************************************/
/****************************************/

int CFootBotMarching::GetDegree(){
   //~ return tPackets.size();
   return degree;
}

/****************************************/
/****************************************/

void CFootBotMarching::Decision() {
	int degreeLocal = tPackets.size();

	/* ======================= LISTEN TO NEIGHBORS ======================= */
	/* ======================= + UPDATE THE SPEED ======================== */
	/* =================================================================== */
	/*	
	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
	int degreeLocal = tPackets.size();
	*/

	if (degreeLocal == 0 && GetDegree() > 0)
	{
		LOG << m_unID << " ";
	}

	Real velocity = 0.0, avgVelocity = 0.0;
	f_fracLeft = 0.0;
	f_fracRight = 0.0;

	for (size_t i = 0; i < degreeLocal; i++) 
	{
		velocity = tPackets[i].Data[2] * 255 + tPackets[i].Data[3];
		velocity = velocity / 100.0;
		if (tPackets[i].Data[4] == 1)
		{ 
			velocity = velocity * (-1.0); 
			f_fracLeft += 1.0; 
		}
		else
		{ 
			f_fracRight += 1.0; 
		}
		avgVelocity += velocity;
	}		
	if (degreeLocal > 0)
	{ 
		avgVelocity = avgVelocity / (1.0*degreeLocal); 
	}

	// Ilja explained the dividing of this value as making it so that the values do not fluctuate around zero but stay better in the range
	//if (avgVelocity > 0)
	if (avgVelocity > 0.0) // Equivalent to line above
	{
		avgVelocity++;
		avgVelocity = avgVelocity / 2;
	}
	else if (avgVelocity < 0)
	{
		avgVelocity--;
		avgVelocity = avgVelocity / 2;
	}

	// ???? relation to spontanenous switching?
	avgVelocity += m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) * 2.0 - 1.0; // -1.0 -> 1.0

	if(!m_sMotionParams.b_isStopped)
	{ 
		m_sMotionParams.SetWheelsVelocity(avgVelocity); 
	}
	/* =================================================================== */
	/* =================================================================== */

	/* ======================= ADJUST RANGE BASED ON CHANGE DETECTION ======================== */
	/* ======================================================================================= */
	if(avgVelocity < 0) // Similar to for loop above
	{ 
		f_fracLeft += 1.0; 
	}
	else 
	{
		f_fracRight += 1.0;
	}

	if (degreeLocal > 0)
	{
		f_fracLeft = f_fracLeft / (1.0 * degreeLocal + 1.0);
		f_fracRight = (1.0 - f_fracLeft);

		// Difference between fractions comparison with 0.5?
		// Or is it, generate one random number, then use option one or two 
		// Or both?
		// Either way, probabilistic is the original code
		// Note: duplicate code

		// TODO: revert back to default probabilistic
		Real fraction_difference = Abs(f_fracLeft - f_fracRight);
		if (mRangeDecisionState == CFootBotMarching::ERangeDecisionMakingState::Probabilistic)
		{
			if (m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) > fraction_difference)
			{
				mInterestFile << timer <<  ": Increasing range due to RNG > fraction_difference(" << fraction_difference << ")\n";
				
				IncreaseRange();
			}
			else if (m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)) < fraction_difference)
			{
				mInterestFile << timer <<  ": Decreasing range due to RNG < fraction_difference(" << fraction_difference << ")\n";
				
				DecreaseRange();
			}
		}
		else if (mRangeDecisionState == CFootBotMarching::ERangeDecisionMakingState::Binary)
		{
			Real rng = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
			if (fraction_difference < rng)
			{
				IncreaseRange();
			}
			else
			{
				DecreaseRange();
			}
		}
	}
	else if (b_breakdown)
	{ 
		mInterestFile << timer <<  ": Increasing range due to detected breakdown (no neighbours)\n";
		
		IncreaseRange();
	}
	// Need to test this if this affects the isolated nodes
	// Otherwise asymmetry
	else if (mRangeSymmetric)
	{
		DecreaseRange();
	}

	f_fracLeft_Change = Abs(f_fracLeft - f_fracLeft_Old);
	f_fracRight_Change = Abs(f_fracRight - f_fracRight_Old);

	f_fracLeft_Old = f_fracLeft;
	f_fracRight_Old = f_fracRight;
	/* ======================================================================================= */
	/* ======================================================================================= */

	/* ======================= SPONTANEOUS SWITCH ======================== */
	// At random a footbot may decide to move in the other direction
	if(m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.SpontSwitchingProb)
	{
		mInterestFile << timer <<  ": Spontaneous switch of direction\n";

		SwitchState();
	}
	/* =================================================================== */
}

/****************************************/
/****************************************/

CCI_DifferentialSteeringActuator* CFootBotMarching::GetWheels() {
	return m_pcWheels;
}

/****************************************/
/****************************************/

void CFootBotMarching::ControlStep() 
{
  	if (mIsNodeOfInterest && !mInterestFile.is_open())
	{
		mInterestFile.open("/mnt/c/argos/pl_check_kit/pl_check_kit/nodeinfo.log", std::ofstream::trunc | std::ofstream::out);
		mInterestFile << "Starting interest ouput for node " << m_unID << std::endl;
	}
   
   /*
    * ATTENTION: m_pcRABA->SetData(size_t,UInt8)
    * That means, in order to make fbID = m_unCounter which are > 255
    * compatible with UInt8, the m_unCounter is composed into the
    * number of times 255 is insider, plus rest.
    * When the message is read inside the loop_functions, it is read like this:
    * m_unFBMsg = tPacketsLOG[i].Data[0]*255 + tPacketsLOG[i].Data[1];
   */
	std:: string strID = GetId().substr (2,5);
	int unID = std::stoi (strID,nullptr,10);
	UInt8 rest = unID % 255;
	UInt8 multiplier = (unID - rest) / 255;
	m_pcRABA->SetData(0,multiplier);
	m_pcRABA->SetData(1,rest);
	timer++;
	b_rabChanged = false;
	b_breakdown = false;
	
	/* ======================= DETECTING THE CHANGE ======================= */
	/* ==================================================================== */
	Real variance_of_change_left_old = variance_of_change_left;
	Real variance_of_change_right_old = variance_of_change_right;
	
	// AVERAGE
	f_fracLeft_AvgChange = (1.0 * timer - 1.0) * f_fracLeft_AvgChange;
	f_fracLeft_AvgChange += f_fracLeft_Change;
    f_fracLeft_AvgChange = f_fracLeft_AvgChange / (1.0 * timer);
    
    f_fracRight_AvgChange = (1.0 * timer - 1.0) * f_fracRight_AvgChange; 
    f_fracRight_AvgChange += f_fracRight_Change;
    f_fracRight_AvgChange = f_fracRight_AvgChange / (1.0 * timer);
    
    // VARIANCE
    variance_of_change_left = (1.0 *timer - 1.0) * variance_of_change_left;
    variance_of_change_left += (f_fracLeft_AvgChange - f_fracLeft_Change) * (f_fracLeft_AvgChange - f_fracLeft_Change);
    variance_of_change_left /= (1.0 * timer);
    
    variance_of_change_right = (1.0 * timer - 1.0) * variance_of_change_right;
    variance_of_change_right += (f_fracRight_AvgChange - f_fracRight_Change)*(f_fracRight_AvgChange - f_fracRight_Change);
    variance_of_change_right /= (1.0 * timer);
	
	// BREAKDOWN?
	if(variance_of_change_left_old < variance_of_change_left || variance_of_change_right_old < variance_of_change_right)
	{
		b_breakdown = true;
	}
	/* ==================================================================== */
	/* ==================================================================== */
    
    
	/* First Decide what to do, then make the respective turns */
	/* ======================= DECISION ======================= */
	Decision();
	/* ======================================================== */
	
	
	/* ======================= MOTION PROCESSES ======================= */
	/* ================================================================ */
	Real vel = m_sMotionParams.m_fWheelVelocity;
	Real velOfMotion =  m_sMotionParams.velFactor*vel;
	if(m_sMotionParams.b_isStopped){ vel = m_sMotionParams.m_fWheelDefaultVelocity; }
	
	int velInt = (int) (Abs(vel) * 100);
	rest = velInt % 255;
	multiplier = (velInt - rest) / 255;
	m_pcRABA->SetData(2,multiplier);
	m_pcRABA->SetData(3,rest);
	if(vel < 0){m_pcRABA->SetData(4,1);}
	else{m_pcRABA->SetData(4,0);}
	
		
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for(size_t i = 0; i < tProxReads.size(); ++i) {
	  cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}
	cAccumulator /= tProxReads.size();
	/* If the angle of the vector is small enough and the closest obstacle
	* is far enough, continue going straight, otherwise curve a little
	*/
	CRadians cAngle = cAccumulator.Angle();
	const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
	
	CVector2 cLightAccumulator;
	for(size_t i = 0; i < tLightReads.size(); ++i) {
	  cLightAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
	}
	
	Real tReadingsDegrees = ToDegrees(cLightAccumulator.Angle()).GetValue() * velOfMotion/Abs(velOfMotion);
	
	bool collAvoiding = false;
	
	if(m_sMotionParams.m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
	  cAccumulator.Length() < m_sMotionParams.m_fDelta ) {
	  /* Go straight */
	  m_pcWheels->SetLinearVelocity(velOfMotion, velOfMotion);
	}
	else {
	   collAvoiding = true;
	  /* Turn, depending on the sign of the angle */
	  if(cAngle.GetValue() > 0.0f) {
		 m_pcWheels->SetLinearVelocity(velOfMotion, 0.0f);
	  }
	  else {
		 m_pcWheels->SetLinearVelocity(0.0f, velOfMotion);
	  } 
	}
	
	Real cHeadingAngle = cDegGoal.GetValue() * velOfMotion/Abs(velOfMotion);
	
	if(!collAvoiding){
	   /* If need to switch, then be turning until the angle to the light beacon is close to the defined degrees */
	   if(Abs(cHeadingAngle - tReadingsDegrees) > 5){
			   /* Wheel speeds based on current turning state */
			   Real fBaseAngularWheelSpeed = velOfMotion;
			   
			   Real fLeftWheelSpeed, fRightWheelSpeed;
			   if(cHeadingAngle > 0) {
				if(tReadingsDegrees > cHeadingAngle) {
					  // Turn Left 
					  fLeftWheelSpeed  = 0.0f;
					  fRightWheelSpeed = fBaseAngularWheelSpeed;
				    }
				else if(tReadingsDegrees <= cHeadingAngle-180) {
					  // Turn Left 
					  fLeftWheelSpeed  = 0.0f;
					  fRightWheelSpeed = fBaseAngularWheelSpeed;
					}
				else{
					  // Turn Right
					  fLeftWheelSpeed  = fBaseAngularWheelSpeed;
					  fRightWheelSpeed = 0.0f;
					}
			   }
			   else if(cHeadingAngle <= 0) {
				if(tReadingsDegrees < cHeadingAngle) {
					  // Turn Right
					  fLeftWheelSpeed  = fBaseAngularWheelSpeed;
					  fRightWheelSpeed = 0.0f;
				    }
				else if(tReadingsDegrees > cHeadingAngle+180) {
					  // Turn Right
					  fLeftWheelSpeed  = fBaseAngularWheelSpeed;
					  fRightWheelSpeed = 0.0f;
					}
				else{
					  // Turn Left 
					  fLeftWheelSpeed  = 0.0f;
					  fRightWheelSpeed = fBaseAngularWheelSpeed;
					}
			   }
	
			m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
		}
		else{
			m_pcWheels->SetLinearVelocity(velOfMotion, velOfMotion);
		}
	}
	cDegGoal.SetValue(90);
	/* ================================================================ */
	/* ================================================================ */
		
	// FINALLY, SET THE COLOR
    if(velOfMotion > 0){ m_pcLEDs->SetAllColors(CColor::GREEN); }
	else if(velOfMotion < 0){m_pcLEDs->SetAllColors(CColor::RED); }
	else{ m_pcLEDs->SetAllColors(CColor::BLACK); }
	
	// Keep track of state at this timestep
	mCurrentHistoryState.Degree = this->GetDegree(); // Need to check if this is recalculated between ControlStep and PostStep
	mCurrentHistoryState.DirectionDecision = this->GetVelocity() > 0.0;
	mCurrentHistoryState.Range = this->GetNewRABRange();
    mCurrentHistoryState.NearestNeighbourDistance = this->GetNNSquaredDistance();

	mHistoryData.push_back(mCurrentHistoryState);
	mCurrentHistoryState.Reset();
}


// Can't get CMake to compile with -std=c++17 flag, requires reinstall and I;m too tired for this
void Clamp(Real& value, const Real l, const Real u)
{
	if (l > u)
	{
		LOG << "Calling Clamp with wrong parameters";
	}

	if (value < l)
	{
		value = l;
	}

	if (value > u)
	{
		value = u;
	}
}

void CFootBotMarching::IncreaseRange()
{
	b_rabChanged = true;
	
	newRABRange += mRangeStep;
	//std::clamp(newRABRange, mRangeLowerBound, mRangeUpperBound); TODO
	Clamp(newRABRange, mRangeLowerBound, mRangeUpperBound);
}



void CFootBotMarching::DecreaseRange()
{
	b_rabChanged = true;

	newRABRange -= mRangeStep;
	//std::clamp(newRABRange, mRangeLowerBound, mRangeUpperBound); TODO
	Clamp(newRABRange, mRangeLowerBound, mRangeUpperBound);
}



/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotMarching, "footbot_marching_controller")
