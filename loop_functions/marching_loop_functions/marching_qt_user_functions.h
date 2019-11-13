#ifndef MARCHING_QT_USER_FUNCTIONS_H
#define MARCHING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CMarchingLoopFunctions;

class CMarchingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CMarchingQTUserFunctions();

   virtual ~CMarchingQTUserFunctions() {}

   virtual void Init(TConfigurationNode& t_tree) override;

   virtual void DrawInWorld();

   void Draw(CFootBotEntity& c_entity);
   
protected:
   CColor DetermineTextColor(const std::string& color);

private:

   void DrawLinks(const std::vector<CVector3>& neighborPositions);

   void DrawInfo(CFootBotEntity& c_entity, std::vector<std::string>& info);
   
private: 
	
   CMarchingLoopFunctions& m_cMarchLF;

   bool mIsEnabled   = false;
   bool mUseIDs      = false;
   bool mUseDegrees  = false;
   bool mUseRanges   = false;

   CColor mColorForText = CColor::BLACK;
   CColor mColorForHubs;
   CColor mColorForHighRange;
   CColor mColorForLowRange;
};

#endif
