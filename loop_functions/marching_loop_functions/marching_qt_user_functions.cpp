#include "marching_qt_user_functions.h"
#include "marching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <controllers/footbot_marching/footbot_marching.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/math/ray3.h>

using namespace argos;

/****************************************/
/****************************************/

CMarchingQTUserFunctions::CMarchingQTUserFunctions() :
   m_cMarchLF(dynamic_cast<CMarchingLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) 
{
	RegisterUserFunction<CMarchingQTUserFunctions,CFootBotEntity>(&CMarchingQTUserFunctions::Draw);
}


CColor CMarchingQTUserFunctions::DetermineTextColor(const std::string& color)
{
	CColor result = CColor::BLACK;
	result.Set(color);
	return result;
}


void CMarchingQTUserFunctions::Init(TConfigurationNode& t_tree)
{
	TConfigurationNode& tDrawingInfo = GetNode(t_tree, "drawing");
	GetNodeAttribute(tDrawingInfo, "isEnabled", mIsEnabled);
	
	TConfigurationNode& tGeneral = GetNode(t_tree, "general");
	GetNodeAttribute(tGeneral, "isEnabled", mIsGeneralEnabled);
	GetNodeAttribute(tGeneral, "useID", mUseIDs);
	GetNodeAttribute(tGeneral, "useDegrees", mUseDegrees);
	GetNodeAttribute(tGeneral, "useRanges", mUseRanges);

	std::string text_color;
	GetNodeAttribute(tGeneral, "hubColor", text_color);
	mColorForHubs = DetermineTextColor(text_color);

	GetNodeAttribute(tGeneral, "highRangeColor", text_color);
	mColorForHighRange = DetermineTextColor(text_color);

	GetNodeAttribute(tGeneral, "lowRangeColor", text_color);
	mColorForLowRange = DetermineTextColor(text_color);

	TConfigurationNode& tPosition = GetNode(t_tree, "position");
	GetNodeAttribute(tPosition, "isEnabled", mIsPositionEnabled);

	TConfigurationNode& tNN = GetNode(t_tree, "nn");
	GetNodeAttribute(tNN, "isEnabled", mIsNNEnabled);
}



void CMarchingQTUserFunctions::Draw(CFootBotEntity& c_entity) 
{
	if (mIsEnabled)
	{
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(c_entity.GetControllableEntity().GetController());
		std:: string strID = c_entity.GetId().substr (2,5);
		int unID = std::stoi (strID,nullptr,10);
   		/* This one is helpful for drawing the ID above the robot */
		/* Alternatively, one could also use a modified DrawInfo function below */
		//~ if(unID == 200){	
	   	//~ DrawText(CVector3(0.0, 0.0, 0.3),   // position
			//~ c_entity.GetId().substr (2,5)); // text
		//~ }
	
		//assert(unID == cController.GetID());

		if (mIsGeneralEnabled)
		{
			// Draw debug info
			std::vector<std::string> info;
			info.push_back(std::to_string(unID));
			info.push_back(std::to_string(cController.GetDegree()));
			info.push_back((cController.IsPotentialHub() || (unID == 7)) ? "hub" : "nohub"); //  || (unID == 3)uncomment unID check if the zero node bug happens again
			if (cController.IsPotentialHighRange())
			{
				info.push_back("high");
			}
			else if (cController.IsPotentialLowRange())
			{
				info.push_back("low");
			}
			else
			{
				info.push_back("avg");
			}
		
			DrawInfo(c_entity, info);
			/////
		}

		if (mIsPositionEnabled)
		{
			// Draw world position of entity
			std::string infoToDraw;
			infoToDraw.append(std::to_string(cController.GetWorldPosition().GetX()));
			infoToDraw.append(", ");
			infoToDraw.append(std::to_string(cController.GetWorldPosition().GetY()));
			DrawText(CVector3(0.0, 0.0, 0.3), infoToDraw, CColor::BLACK);
			/////
		}
		
		if (mIsNNEnabled)
		{
			std::string info;
			//info.append("[").append(std::to_string(unID)).append("]");
			info.append(std::to_string(unID));
			DrawText(CVector3(0.0, 0.0, 0.3), info, CColor::BLACK);

			// Draw circle of neighbourhood
			Real distance = sqrt(cController.GetNNSquaredDistance());
			Real z = 0.05;
			DrawCircle(CVector3(0.0, 0.0, z), CQuaternion()/*.FromEulerAngles(CRadians::ZERO, CRadians::PI_OVER_TWO, CRadians::ZERO)*/, distance, CColor::YELLOW, false);
			DrawCircle(CVector3(0.0, 0.0, z), CQuaternion(), cController.GetNewRABRange(), CColor::RED, false);
			/////

			/*Real diameter = 0.25;
			CColor color = CColor::BLACK;
			CFootBotMarching::EDistanceState distance_state = cController.GetDistanceState();
			if (distance_state == CFootBotMarching::EDistanceState::ISOLATED)
				color = CColor::ORANGE;
			else if (distance_state == CFootBotMarching::EDistanceState::CLUSTERED)
				color = CColor::BLUE;

			//DrawPoint(CVector3(0.0, 0.0, z), color, diameter);
			DrawCircle(CVector3(0.0, 0.0, z), CQuaternion(), diameter, color, true);*/
		}	
	}
}



void CMarchingQTUserFunctions::DrawInWorld() 
{
   /* Go through all the robot neighbors and draw links to them */
   for(CMarchingLoopFunctions::TNeighborsMap::const_iterator it = m_cMarchLF.GetNeighborPositions().begin();
       it != m_cMarchLF.GetNeighborPositions().end();
       ++it) {
      //~ DrawLinks(it->second);
   }
}



void CMarchingQTUserFunctions::DrawLinks(const std::vector<CVector3>& neighborPositions) 
{
   /* Start drawing if a footbot has at least one neighbor */
   if(neighborPositions.size() > 1) 
   {
      size_t unStart = 0;
      size_t unEnd = 1;
      CVector3 pos0 = neighborPositions[unStart];
      pos0.SetZ(0.3f);
      while(unEnd < neighborPositions.size()) 
      {
		   CVector3 pos1 = neighborPositions[unEnd];
		   pos1.SetZ(0.3f);
         DrawRay(CRay3(pos1,pos0), CColor::GREEN);
         ++unEnd;
      }
   }
}



void CMarchingQTUserFunctions::DrawInfo(CFootBotEntity& c_entity, std::vector<std::string>& info)
{
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   /*if (info.size() > 0) 
   {
      size_t unEnd = 1;
      std::string infoToDraw = info[unEnd-1];
      while(unEnd < info.size())
      {
         infoToDraw = infoToDraw + " " + info[unEnd];
         ++unEnd;
      }
	   //~ DrawText(c_entity.GetEmbodiedEntity().GetOriginAnchor().Position,   // position
	   DrawText(CVector3(0.0, 0.0, 0.3), infoToDraw); // position, text, color
   }*/

   	std::string infoToDraw;
   	if (mUseIDs)
   	{
		infoToDraw.append("[").append(info[0]).append("] ");
   	}
   	if (mUseDegrees)
   	{
		infoToDraw.append(info[1]);
   	}

	// If it has been marked as a potential hub, we wish to make them pop out more
	CColor color = mColorForText;
	
	bool isHub = info[2] == "hub";
	bool isHighRange = info[3] == "high";
	bool isLowRange = info[3] == "low";  
	
	if (isHub)
	{
		if (isHighRange)
		{
			color = CColor::RED;
		}
		else if (isLowRange)
		{
			color = CColor::YELLOW;
		}
		else
		{
			color = CColor::ORANGE;
		}
	}
	else
	{
		if (isHighRange)
		{
			color = CColor::BLUE;
		}
		else if (isLowRange)
		{
			color = CColor::GREEN;
		}
		else
		{
			color = CColor::CYAN;
		}
	}

	DrawText(CVector3(0.0, 0.0, 0.3), infoToDraw, color);
}



REGISTER_QTOPENGL_USER_FUNCTIONS(CMarchingQTUserFunctions, "marching_qt_user_functions")