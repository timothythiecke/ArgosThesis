#include "marching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <controllers/footbot_marching/footbot_marching.h>
#include <stdio.h>  /* defines FILENAME_MAX */
#include <unistd.h>
#define GetCurrentDir getcwd

#include <algorithm>

/****************************************/
/****************************************/

CMarchingLoopFunctions::CMarchingLoopFunctions() :
   m_pcRNG(NULL),
   G(10,std::vector<int>(10)),
   degDistTot(10),
   m_bShowLinks(false) {
}

/****************************************/
/****************************************/

void CMarchingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
	  m_pcRNG = CRandom::CreateRNG("argos");
	  
	  TConfigurationNode& tDir = GetNode(t_node, "folder");
      GetNodeAttribute(tDir, "name", str_dirName);
      GetNodeAttribute(tDir, "randID", s_randID);
      
	  	TConfigurationNode& tOutput = GetNode(t_node, "output");
		GetNodeAttribute(tOutput, "timer", mOutputTimer);

		TConfigurationNode& tHubs = GetNode(t_node, "hubs");
		GetNodeAttribute(tHubs, "fraction", mHubMarkingFraction);

	   OpenOutFilesID();
	   finished = false;
	   timer = 0;
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CMarchingLoopFunctions::Reset() {
   finished = false;
   os_degD.close();
   os_degD_Tot.close();
   OpenOutFilesID();
   timer = 0;
}

/****************************************/
/****************************************/

void CMarchingLoopFunctions::Destroy() {   
	timer = GetSpace().GetSimulationClock();
	
#define DIRECT_OUTPUT // If defined, outputs files directly to the pl_check_kit folder
#ifdef DIRECT_OUTPUT
	std::ofstream fDegrees;
	std::ofstream fDegreesTot;
	std::ofstream fMetaData;
	std::ofstream fLog; // I believe ARGoS does not output anything to log files, so this helps to debug
	std::ofstream fDegreesGC;
	std::ofstream fDegreesTotGC;

	// TODO: this should use the function that Ilja provided, but has been rewritten for convenience
	fDegrees.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution.dat", std::ofstream::trunc | std::ofstream::out);
	fDegreesTot.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution_tot.dat", std::ofstream::trunc | std::ofstream::out);
	fMetaData.open("/mnt/c/argos/pl_check_kit/pl_check_kit/metaData.dat", std::ofstream::trunc | std::ofstream::out);
	fLog.open("/mnt/c/argos/pl_check_kit/pl_check_kit/log.log", std::ofstream::trunc | std::ofstream::out);
	fDegreesGC.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistributionGiant.dat", std::ofstream::trunc | std::ofstream::out);
	fDegreesTotGC.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution_totGiant.dat", std::ofstream::trunc | std::ofstream::out);

	if (!fDegrees.is_open() || !fDegreesTot.is_open() || !fMetaData.is_open() || !fLog.is_open()) // TODO
	{
		// TODO: replace with a throw
		LOG << "One of the files could not be opened! Fix me first" << std::endl;
	}

	std::vector<int> degrees;
   	std::vector<double> degrees_tot;
#endif
	// Export the degree distribution from degDistTot
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

 
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) 
	{
		// Create a pointer to the current foot-bot 
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		std:: string strID = cController.GetId().substr (2,5);
		int unID = std::stoi (strID,nullptr,10);
		
#ifdef DIRECT_OUTPUT
		degrees.push_back(cController.GetDegree());
		degrees_tot.push_back(degDistTot[unID] * 1.0 / (timer*1.0));
#else
		os_degD << cController.GetDegree() << std::endl;
		os_degD_Tot << degDistTot[unID]*1.0/(timer*1.0) << std::endl;
#endif
	}
   
#ifdef DIRECT_OUTPUT
	// Transform the decorated graph
	std::vector<Node> decorated;
	this->DecorateGraph(decorated);
	fLog << "Decorated graph G. Expected size: " << G.size() << " Got: " << decorated.size() << "\n"; 

	std::vector<std::vector<int>> components;
	if (!decorated.empty())
	{
		fLog << "Determining components...\n";
		
		for (int i = 0; i < decorated.size(); i++)
		{
			if (!decorated[i].visited)
			{
				// Every time this hits, it means we discovered a new component
				components.resize(components.size() + 1); // Therefore a new component vector should be added
				this->DetermineComponents(decorated, components, decorated[i]); // Start with the root of this new component
			}
		}
		fLog << "Done with determining components! Found " << components.size() << " components\n";
	}
	else
	{
		fLog << "Decorated graph is empty! This should never happen!!\n";
	}

	// Sort components by size (descending)
	std::sort(components.begin(), components.end(), [](const std::vector<int>& lhs, const std::vector<int>& rhs)
	{
		return lhs.size() > rhs.size();
	});

	// Output the degree distributions of the largest component
	for (const int i : components[0])
	{
		fDegreesGC << degrees[i] << std::endl;
		fDegreesTotGC << degrees_tot[i] << std::endl;
		// Note, this assumes that both files will not have zeroes, this should be the case considering a node part of the component should have at LEAST 1 connection 
	}

	// Prepare data for files and close file handles
	std::sort(degrees.begin(), degrees.end(), [](const int i, const int j)
	{
		return i < j;
	});

	std::sort(degrees_tot.begin(), degrees_tot.end(), [](const double i, const double j)
	{
		return i < j;
	});
		
	// Populate the degree distribution files
	int degrees_omitted = 0;
	for (const int i : degrees)
	{
		if (i > 0)
		{
			fDegrees << i << std::endl;
		}
		else
		{
			degrees_omitted++;
		}
	}

	int degrees_tot_omitted = 0;
	for (const double d : degrees_tot)
	{
		if (d > 0.0) // Floating point warning!
		{
			fDegreesTot << d << std::endl;
		}
		else
		{
			degrees_tot_omitted++;
		}
	}

	// Populate metadata file
	fMetaData << "Simulation ran for " << timer << "s with population of size " << degrees.size() << std::endl;
	fMetaData << "Omitted " << degrees_omitted << " degree counts\nOmitted " << degrees_tot_omitted << " average degree counts" << std::endl;
	fMetaData << "Largest component contained " << components[0].size() << " elements (" << components[0].size() / (double)(degrees.size()) << ")" << std::endl;

	fLog << "Done with writing, closing all file handles..." << std::endl;

	// Close file handles
	fDegrees.close();
	fDegreesTot.close();
	fMetaData.close();
	fLog.close();
	fDegreesGC.close();
	fDegreesTotGC.close();
#endif
	// The file handles should be closed regardless, as they are created at the start
	os_degD.close();
	os_degD_Tot.close();
}



// Transforms graph G into something that allows us to DFS it in order to build components
void CMarchingLoopFunctions::DecorateGraph(std::vector<Node>& decoratedGraph)
{
	int i = 0;
	for (const std::vector<int>& v : G)
	{
		Node node;
		node.index = i++; 
		node.connections = v; // copy

		decoratedGraph.push_back(node);
	}
}



// Determines which components exist
void CMarchingLoopFunctions::DetermineComponents(std::vector<Node>& decoratedGraph, std::vector<std::vector<int>>& components, Node& nodeToVisit)
{
	nodeToVisit.visited = true;
	components.back().push_back(nodeToVisit.index);

	for (const int i : nodeToVisit.connections)
	{
		if (!decoratedGraph[i].visited)
		{
			this->DetermineComponents(decoratedGraph, components, decoratedGraph[i]);
		}
	}
}



/****************************************/
/****************************************/

void CMarchingLoopFunctions::PreStep() {
	int newNode;
	
	// Loop over the swarm and update the RAB range for every robot
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		   it != m_cFootbots.end();
		   ++it) {
		  // Create a pointer to the current foot-bot 
		  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		  CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		  std:: string strID = cController.GetId().substr (2,5);
		  int unID = std::stoi (strID,nullptr,10);
		 
		// And let the robots adjust their communication range:
		if(cController.IsRABChanged()){
			CRABEquippedEntity& cFootBotRAB = cFootBot.GetRABEquippedEntity();
			cFootBotRAB.SetRange(cController.GetNewRABRange());
		}
		
		// First clear the old list of network connections
		G[unID].clear();

		// TODO: need to clear G2
	}
	
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		   it != m_cFootbots.end();
		   ++it) {
		  // Create a pointer to the current foot-bot 
		  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		  CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		  std:: string strID = cController.GetId().substr (2,5);
		  int unID = std::stoi (strID,nullptr,10);
		
		// Copy the list of network connections from the data of the range-and-bearing sensor into G
		// Add local links from range-and-bearing sensors			
		CCI_RangeAndBearingSensor::TReadings tPackets = cController.GetTPackets();
		for(size_t i = 0; i < tPackets.size(); ++i) {
			newNode = tPackets[i].Data[0]*255 + tPackets[i].Data[1];
			if(std::find(G[unID].begin(), G[unID].end(), newNode) == G[unID].end() 
			&& std::find(G[newNode].begin(), G[newNode].end(), unID) == G[newNode].end()
			){
				G[unID].push_back(newNode);
				G[newNode].push_back(unID); // Comment out this line to have directed
				// Dummy graph along with the G, where you have both lines, don't use that other graph for the communication of the T packets
			}
		}
	}
	
		
	for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
	{
		// Create a pointer to the current foot-bot 
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		std::string strID = cController.GetId().substr (2,5);
		int unID = std::stoi (strID,nullptr,10);
		  
		// --- UPDATE INFORMATION RECEIVED FROM NEIGHBORS ---
		CCI_RangeAndBearingSensor::TReadings newTPackets;
		CCI_RangeAndBearingSensor::SPacket newSPacket;
		
		for(int j = 0; j < G[unID].size(); j++)
		{
			newSPacket.Data.Resize(5);
			// --- Get the ID of the node --- 
			newNode = G[unID][j];
			
			// --- Save the ID of the node in the fakeSensor --- 
			UInt8 rest = newNode % 255;
			UInt8 multiplier = (newNode - rest) / 255;
			newSPacket.Data[0] = multiplier;
			newSPacket.Data[1] = rest;
			
			// --- Save the velocity of the node in the fakeSensor --- 
			int newVelocity = (int) round (Abs(cController.GetVelocity()) * 100);
			rest = newVelocity % 255;
			multiplier = (newVelocity - rest) / 255;
			newSPacket.Data[2] = multiplier;
			newSPacket.Data[3] = rest;
			if (cController.GetVelocity() < 0)
			{
				newSPacket.Data[4] = 1;
			}
			else 
			{
				newSPacket.Data[4] = 0;
			}
			
			// --- Add the fake newS to the vector of all readings --- 
			newTPackets.push_back(newSPacket);
			newSPacket.Data.Clear();
		}
		cController.SetFakeTPackets(newTPackets);
	}
}

/****************************************/
/****************************************/

void CMarchingLoopFunctions::PostStep() {
   	Real avgRABRange = 0.0, avgDegree = 0.0;
   	int currentTime = GetSpace().GetSimulationClock();
   	std::vector<int> degDist;
   	std::vector<CFootBotMarching*> controllers;

   	// Loop over the swarm and record the degree of every robot
   	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
   	{
		// Create a pointer to the current foot-bot
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
	  
		// Get the foot-bot controller
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		std:: string strID = cController.GetId().substr (2,5);
		int unID = std::stoi (strID,nullptr,10);
      
    	avgDegree += cController.GetDegree();
    	avgRABRange += cController.GetNewRABRange();
      
    	degDist.push_back(cController.GetDegree());
		
		cController.MarkPotentialHub(false);
		controllers.push_back(&cController);
		
    	//LOG << cController.GetDegree() << "; ";
		
		// This will output the directed information
		// degDist.push_back(G2[unID].size());

    	// Alternative code for displaying the comm links
		//~ for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
				//~ it != m_cFootbots.end();
				//~ ++it) {
				//~ // Create a pointer to the current foot-bot
				//~ CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
			  
				//~ // Clear the list of positions
				//~ m_tNeighbors[pcFB].clear();
		//~ }
	}
	//LOG << std::endl;

	Real activeRobots = m_cFootbots.size() * 1.0;

	avgRABRange = avgRABRange / activeRobots;
	avgDegree = avgDegree / activeRobots;

	// If the degree of the footbot is in the top x%, then mark it as a potential hub
	// In order for this to work, we need the deg distribution array as is
	// We need a mapping between the index and its degree even after sorting
	// Sort descending by grade
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		if (lhs != nullptr && rhs != nullptr)
		{
			return lhs->GetDegree() > rhs->GetDegree();
		}
		return false;
	});
	
	// Then for the top x, mark them as being a potential hub
	int i = controllers.size() - 1;
	int counter = 0;
	for (CFootBotMarching* ptr : controllers)
	{
		if (ptr != nullptr)
		{
			if (counter < (controllers.size() * mHubMarkingFraction))
			{
				ptr->MarkPotentialHub(true);
				counter++;
			}

			// Note, this does the same thing (hopefully) as the sort write code below out of this for loop
			// But will avoid another sort call
			degDistTot[controllers.size() - 1 - i] += controllers[i]->GetDegree();
			i--;
		}
	}

	// Avoid sorting a second time
	/*std::sort (degDist.begin(), degDist.end());
	for(int i = 0; i < degDist.size(); i++)
   	{
		degDistTot[i] += degDist[i]; 
	}*/
   
	if(currentTime % mOutputTimer == 0)
	{ 
		LOG << GetSpace().GetSimulationClock() << "\t" << avgDegree << std::endl; 
	}
}

/****************************************/
/****************************************/

bool CMarchingLoopFunctions::sortFunction (int i,int j) { 
	return (i>j); 
}

/****************************************/
/****************************************/

bool CMarchingLoopFunctions::IsExperimentFinished() {
	return finished;
}

/****************************************/
/****************************************/

void CMarchingLoopFunctions::OpenOutFilesID() {
    // This method automatically detects the current working directory and opens the output files
    std::string filename = "";
    std::string dirID = GetCurrentWorkingDir();
    dirID = dirID.substr(0, dirID.find("argos3"));
    if(dirID=="") LOGERR << "Working directory not found. Please make sure you saved argos3_simulator in the ~/argos3/... directory." << std::endl;

    dirID.append("argos3/argos3-examples/experiments/results/");
    dirID.append(str_dirName).append("/");
    dirID.append(s_randID).append("/");

    filename.append(dirID).append("degDistribution.dat");
    os_degD.open(filename, std::ios_base::trunc | std::ios_base::out);
    filename="";

    filename.append(dirID).append("degDistribution_tot.dat");
    os_degD_Tot.open(filename, std::ios_base::trunc | std::ios_base::out);
    
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    degDistTot.resize(m_cFootbots.size());
    G.resize(m_cFootbots.size());
    
    // Initialize the controller with the correct range-and-bearing setting and set all elements of degDistTot to 0
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		   it != m_cFootbots.end();
		   ++it) {
		  // Create a pointer to the current foot-bot 
		  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		  CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		  
		  CRABEquippedEntity& cFootBotRAB = cFootBot.GetRABEquippedEntity();
		  cController.SetNewRABRange(cFootBotRAB.GetRange());
		  
		  std:: string strID = cController.GetId().substr (2,5);
		  int unID = std::stoi (strID,nullptr,10);
		  
		  degDistTot[unID] = 0;
		  G[unID].resize(0);
	}
}

/****************************************/
/****************************************/

std::string CMarchingLoopFunctions::GetCurrentWorkingDir() {
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMarchingLoopFunctions, "marching_loop_functions")
