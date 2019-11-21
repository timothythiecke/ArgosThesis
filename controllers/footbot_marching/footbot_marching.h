/* 
 * An example marching controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/locust_marching.argos
 */

#ifndef FOOTBOT_MARCHING_H
#define FOOTBOT_MARCHING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

#include <fstream>

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotMarching : public CCI_Controller {

public:

   /* Class constructor. */
   CFootBotMarching();

   /* Class destructor. */
   virtual ~CFootBotMarching() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_marching_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);
   
   void SetWheelSpeedsFromVector();
   
   bool IsClockwise() {
      return b_clockwise;
   }
   
   /*
    * This function is called once every ControlStep.
    * It describes the decision-making procedures.
    */
   void Decision();

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() 
   {
      mInterestFile.close();
   }
   
   CCI_RangeAndBearingSensor::TReadings GetTPackets();
   
   CCI_DifferentialSteeringActuator* GetWheels();
   
   void SwitchState();
   Real GetVelocity();
   bool IsRABChanged();
   Real GetNewRABRange();
   void SetNewRABRange(Real newRange);
   void SetDegree(int newDegree);
   int GetDegree();
   bool GetBreakdown();
   void SetFakeTPackets(CCI_RangeAndBearingSensor::TReadings& fakeTPackets);
   
   void MarkPotentialHub(bool isPotentialHub) { mPotentialHub = isPotentialHub; };
   bool IsPotentialHub() const { return mPotentialHub; }

   void MarkPotentialHighRange(bool isPotentialHighRange) { mPotentialHighRange = isPotentialHighRange; }
   bool IsPotentialHighRange() const { return mPotentialHighRange; }

   void MarkPotentialLowRange(bool isPotentialLowRange) { mPotentialLowRange = isPotentialLowRange; }
   bool IsPotentialLowRange() const { return mPotentialLowRange; }

   int GetID() const { return m_unID; }

   void ResetVisualizationParameters()
   {
      mPotentialHub = false;
      mPotentialHighRange = false;
      mPotentialLowRange = false;
   }

   void SetInterestNode(bool interestingNode) { mIsNodeOfInterest = interestingNode; }
   bool IsInterestingNode() const { return mIsNodeOfInterest; }

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The two possible states in which the controller can be */
      enum EState {
         CLOCKWISE = 0,
         COUNTER_CLOCKWISE
      } State;
      
      /* Probability to spontaneously switch the heading direction */
      Real SpontSwitchingProb;
      /* Used as a range for uniform number generation */
      CRange<Real> ProbRange;
      
      SStateData();
      void Init(TConfigurationNode& t_tree);
      void Reset();
      bool IsClockwise();
   };
   
   /*
    * Contains motion parameters of the controller.
    */
   struct SMotionParams {
	   
	   /* Maximum tolerance for the angle between
	   * the robot heading direction and
	   * the closest obstacle detected. */
	   CDegrees m_cAlpha;
	   /* Maximum tolerance for the proximity reading between
	   * the robot and the closest obstacle.
	   * The proximity reading is 0 when nothing is detected
	   * and grows exponentially to 1 when the obstacle is
	   * touching the robot.
	   */
	   Real m_fDelta;
	   /* Wheel speed. */
	   Real m_fWheelVelocity;
	   Real m_fWheelDefaultVelocity;
	   Real velFactor;
	   bool b_isStopped;
	   int switchedStates;
	   int notSwitchedStates;
	   int avgSwitched;
	   int avgNotSwitched;
	   /* Angle tolerance range to go straight.
	   * It is set to [-alpha,alpha]. */
	   CRange<CRadians> m_cGoStraightAngleRange;
	   
	   SMotionParams();
	   void Init(TConfigurationNode& t_tree);
	   void SaveWheelsVelocity();
	   void SetWheelsVelocity(Real f_velocity);
	   void ReSetWheelsVelocity();
   };

protected:
   virtual void IncreaseRange();
   virtual void DecreaseRange();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_marching_controller> section.
    */
    
    /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SMotionParams m_sMotionParams;
   
   /* Contains the message received from the neighboring foot-bot */
   const CCI_RangeAndBearingSensor::SPacket* m_psFBMsg;
   UInt64 m_unFBMsg;
   
   enum MovingDirection {
      CLOCKWISE = 0,    
      COUNTER_CLOCKWISE  
   } m_eMovingDirection;
   
   /* True if footbot is marching in a clockwise manner */
   bool b_clockwise;
   
   /* The angle to the light beacon that the footbot needs to maintain */
   CDegrees cDegGoal;
   
   /* The random number generator */
   CRandom::CRNG* m_pcRNG;
   
   /* Indicates whether the range-and-bearing range has changed */
   bool b_rabChanged;
   
   bool b_breakdown;
   
   CCI_RangeAndBearingSensor::TReadings tPackets;
   
   int timer;
   
   Real newRABRange;
   
   int m_unID;
   int degree;
   
   // True if we are interested in which events happend for this node
   bool mIsNodeOfInterest = false; 
   std::ofstream mInterestFile;

   Real f_fracLeft;
   Real f_fracRight;
   
   Real f_fracLeft_Old;
   Real f_fracRight_Old;
   
   Real f_fracLeft_Change;
   Real f_fracRight_Change;
   
   Real f_fracLeft_AvgChange;
   Real f_fracRight_AvgChange;
   
   Real variance_of_change_left;
   Real variance_of_change_right;

   // Marks the node as being a potential hub in order to spot it better in the visualization
   bool mPotentialHub = false;
   bool mPotentialHighRange = false;
   bool mPotentialLowRange = false;

   ///////
   // Range parameters
   double mRangeLowerBound = -1.0;
   double mRangeUpperBound = -1.0;
   double mRangeStep       = -1.0;
   // In the original implementation, the code did not have a symmetric DecreaseRange() call and got overshadowed by IncreaseRange() calls, setting this to true will balance these calls
   bool mRangeSymmetric = false;
   
   // Does the range adjustment code use a probabilistic decision making model or does it use a binary state
   enum class ERangeDecisionMakingState
   {
      Invalid = -1,
      Probabilistic = 0,
      Binary
   };
   ERangeDecisionMakingState mRangeDecisionState = ERangeDecisionMakingState::Invalid;
   ///////
};

#endif
