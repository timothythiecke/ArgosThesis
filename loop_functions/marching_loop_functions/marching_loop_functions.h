#ifndef MARCHING_LOOP_FUNCTIONS_H
#define MARCHING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

using namespace argos;

// Helper struct to use for DFS traversal in order to determine the components
struct Node
{
   std::vector<int> connections;
   int index = -1;
   bool visited = false;
};


// Helper struct for metadata mapping between robot index and associated value, less overhead than std::pair
template <typename T>
struct IndexValuePair
{
   int index = -1;
   int timeFrame = -1;
   T value;
};
template <>
struct IndexValuePair<int>
{
   int index = -1;
   int timeFrame = -1;
   int value = -1;
};
template <>
struct IndexValuePair<Real>
{
   int index = -1;
   int timeFrame = -1;
   Real value = -1.0;
};


class CMarchingLoopFunctions : public CLoopFunctions 
{
public:

   CMarchingLoopFunctions();
   virtual ~CMarchingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void PreStep();
   virtual void PostStep();
   virtual bool IsExperimentFinished();
   void OpenOutFilesID();
   std::string GetCurrentWorkingDir();
   bool sortFunction (int i,int j);
   
   std::vector<int> degDistTot;
   std::vector<std::vector<int>> G;
   
   // ----------------------------------------
   // This was used to draw the links.
   // Although it is not implemented in the cpp, the qt_functions still use it, so it should not be removed.
   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TNeighborsMap;
   TNeighborsMap m_tNeighbors;
   
   inline const TNeighborsMap& GetNeighborPositions() const {
      return m_tNeighbors;
   }
   // ----------------------------------------
protected:
   void DecorateGraph(std::vector<Node>& decoratedGraph); // Transforms graph G into something that allows us to DFS it in order to build components
   void DetermineComponents(std::vector<Node>& decoratedGraph, std::vector<std::vector<int>>& components, Node& nodeToVisit);
private:

   CRandom::CRNG* m_pcRNG;

   std::string str_dirName;
   std::string s_randID;
   std::ofstream os_degD;
   std::ofstream os_degD_Tot;
   
   bool finished;
   bool m_bShowLinks;
   
   int mOutputTimer = 50;
   // The fraction of the total amount of robots that will be marked as a potential hub
   double mHubMarkingFraction = 0.0;

   std::vector<Real> mRangeDistTot;
   
   IndexValuePair<int> mHighestDegreeCount;
   IndexValuePair<Real> mHighestAverageDegreeCount;
   IndexValuePair<Real> mHighestRange;
   IndexValuePair<Real> mHighestAverageRange;

   int timer;

   bool mZeroWorkAround = false;
   bool mOutputInformationAboutZeroWorkAround = false;

   int mNodeOfInterest = -1;

   std::vector<Real> mNNSquaredDistanceDistribution;

   void OutputNNDistanceDistribution();
   bool mDrawNNLinks = false;
};

#endif
