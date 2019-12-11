#include "marching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <controllers/footbot_marching/footbot_marching.h>
#include <stdio.h>  /* defines FILENAME_MAX */
#include <assert.h>
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
		GetNodeAttribute(tOutput, "nodeOfInterest", mNodeOfInterest);

		TConfigurationNode& tHubs = GetNode(t_node, "hubs");
		GetNodeAttribute(tHubs, "fraction", mHubMarkingFraction);

		TConfigurationNode& tNode = GetNode(t_node, "node");
		GetNodeAttribute(tNode, "zeroWorkAround", mZeroWorkAround);
		GetNodeAttribute(tNode, "output", mOutputInformationAboutZeroWorkAround);

		TConfigurationNode& tNN = GetNode(t_node, "nearestNeighbour");
		GetNodeAttribute(tNN, "drawLinks", mDrawNNLinks);

		TConfigurationNode& tRepresentative = GetNode(t_node, "representative");
		GetNodeAttribute(tRepresentative, "fraction", mRepresentativeFraction);

	   OpenOutFilesID();
	   finished = false;
	   timer = 0;
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }

   //os_interest << "Starting run for seed " << CSimulator::GetInstance().GetRandomSeed() << " with focus on node " << mNodeOfInterest << std::endl;
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
	
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

#define DIRECT_OUTPUT // If defined, outputs files directly to the pl_check_kit folder
#ifdef DIRECT_OUTPUT
	std::ofstream fDegrees;
	std::ofstream fDegreesTot;
	std::ofstream fMetaData;
	std::ofstream fLog; // I believe ARGoS does not output anything to log files, so this helps to debug
	std::ofstream fDegreesGC;
	std::ofstream fDegreesTotGC;
	std::ofstream fDegreesUnsorted;
	std::ofstream fDegreesTotUnsorted;
	std::ofstream fRanges;
	std::ofstream fRangesTot;

	// TODO: this should use the function that Ilja provided, but has been rewritten for convenience
	fDegrees.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution.dat", std::ofstream::trunc | std::ofstream::out);
	fDegreesTot.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution_tot.dat", std::ofstream::trunc | std::ofstream::out);
	fMetaData.open("/mnt/c/argos/pl_check_kit/pl_check_kit/metaData.dat", std::ofstream::trunc | std::ofstream::out);
	fLog.open("/mnt/c/argos/pl_check_kit/pl_check_kit/log.log", std::ofstream::trunc | std::ofstream::out);
	fDegreesGC.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistributionGiant.dat", std::ofstream::trunc | std::ofstream::out);
	fDegreesTotGC.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution_totGiant.dat", std::ofstream::trunc | std::ofstream::out);
	fDegreesUnsorted.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistributionUnsorted.dat", std::ofstream::trunc | std::ofstream::out);
	fDegreesTotUnsorted.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degDistribution_totUnsorted.dat", std::ofstream::trunc | std::ofstream::out);
	fRanges.open("/mnt/c/argos/pl_check_kit/pl_check_kit/ranges.dat", std::ofstream::trunc | std::ofstream::out);
	fRangesTot.open("/mnt/c/argos/pl_check_kit/pl_check_kit/ranges_tot.dat", std::ofstream::trunc | std::ofstream::out);

	if (!fDegrees.is_open() || !fDegreesTot.is_open() || !fMetaData.is_open() || !fLog.is_open()) // TODO
	{
		// TODO: replace with a throw
		LOG << "One of the files could not be opened! Fix me first" << std::endl;
	}

	std::vector<int> degrees(m_cFootbots.size());
   	std::vector<Real> degrees_tot(m_cFootbots.size());

	std::vector<Real> ranges(m_cFootbots.size());
	std::vector<Real> ranges_tot(m_cFootbots.size());
#endif
	// Export the degree distribution from degDistTot
	std::vector<CFootBotMarching*> controllers(m_cFootbots.size());
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) 
	{
		// Create a pointer to the current foot-bot 
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		std:: string strID = cController.GetId().substr (2,5);
		int unID = std::stoi (strID,nullptr,10);
		
#ifdef DIRECT_OUTPUT
		//degrees.push_back(cController.GetDegree());
		//degrees_tot.push_back(degDistTot[unID] * 1.0 / (timer*1.0));
		degrees[unID] = cController.GetDegree();
		degrees_tot[unID] = degDistTot[unID] * 1.0 / (timer*1.0);

		ranges[unID] = cController.GetNewRABRange();
		ranges_tot[unID] = mRangeDistTot[unID] * 1.0 / (timer*1.0);
#else
		os_degD << cController.GetDegree() << std::endl;
		os_degD_Tot << degDistTot[unID]*1.0/(timer*1.0) << std::endl;
#endif

		cController.CalulateDistanceStateRatios();
		cController.CalculateAverageNNDistance();
		controllers[unID] = &cController;
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

	// Output unsorted, unsanitized info
	int max = degrees.size(); // Note that this should be the same for both!
	for (int i = 0; i < max; i++)
	{
		fDegreesUnsorted << degrees[i] << std::endl;
		fDegreesTotUnsorted << degrees_tot[i] << std::endl;

		// Update metadata information about the highest average degree count
		if (degrees_tot[i] > mHighestAverageDegreeCount.value)
		{
			mHighestAverageDegreeCount.value = degrees_tot[i];
			mHighestAverageDegreeCount.index = i;
			mHighestAverageDegreeCount.timeFrame = timer;
		}
		if (ranges_tot[i] > mHighestAverageRange.value)
		{
			mHighestAverageRange.value = ranges_tot[i];
			mHighestAverageRange.index = i;
			mHighestAverageRange.timeFrame = timer;
		}
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

	// Populate the degree distribution files, omitting any zero values and sorted
	// Do the same for the range distribution files
	int degrees_omitted = 0;
	int degrees_tot_omitted = 0;	
	for (int i = 0; i < max; i++)
	{
		// Degrees
		if (degrees[i] > 0)
		{
			fDegrees << degrees[i] << std::endl;
		}
		else
		{
			degrees_omitted++;
		}
		if (degrees_tot[i] > 0.0)
		{
			fDegreesTot << degrees_tot[i] << std::endl;
		}
		else 
		{
			degrees_tot_omitted++;
		}

		// Ranges
		if (ranges[i] > 0.0)
		{
			fRanges << ranges[i] << std::endl;
		}
		if (ranges_tot[i] > 0.0)
		{
			fRangesTot << ranges_tot[i] << std::endl;
		}
	}

	// What about k-means clustering over 
	// Determine representative for most isolated during entire run
	// Current heuristic uses averaged NN distances over time, this should offer a reliable way of searching for the most
	// isolated node, and maybe the most clustered node but this should be discussed with Ilja
	// Fraction based evaluation seems broken somehow
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		//return lhs->GetIsolatedFraction() > rhs->GetIsolatedFraction(); // see todo in footbot_marching.h
		return lhs->mAverageNNDistance > rhs->mAverageNNDistance;
	});

	vector<std::ofstream> files(5);
	vector<std::string> file_names = { "0range", "1nn", "2dstate", "3dirdec" , "4deg"};

	int c = 0;
	for (std::ofstream& file : files)
	{
		std::string filename;
		filename.append("/mnt/c/argos/pl_check_kit/pl_check_kit/");
		filename.append("0_isolated_");
		filename.append(file_names[c]);
		filename.append(".dat");
		file.open(filename);
		
		c++;
	}
	
	assert(controllers[0] != nullptr);
	std::ofstream isolatedMeta;
	isolatedMeta.open("/mnt/c/argos/pl_check_kit/pl_check_kit/0_isolated_meta.dat");
	isolatedMeta << controllers[0]->GetID();
	isolatedMeta.close();
	
	const vector<CFootBotMarching::SHistoryData>& history = controllers[0]->GetHistory();
	//for (const CFootBotMarching::SHistoryData& history_element : history)
	int d = 0;
	for (c = 0; c < history.size(); c++)
	{
		files[0] << history[c].Range << std::endl;		
		if (c == 0) // Edge case hack, use the nearest neighbour distance of the first nonzero element
			d = 1;
		else
			d = c;
		files[1] << sqrt(history[d].NearestNeighbourDistance) << std::endl;
		files[2] << (int)(history[c].DistanceState) << std::endl;
		files[3] << history[c].DirectionDecision << std::endl;
		files[4] << history[c].Degree << std::endl;
	}

	for (std::ofstream& file : files)
	{
		file.close();
	}

	/*// Determine representative for most clustered during entire run
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		return lhs->GetClusteredFraction() > rhs->GetClusteredFraction();
	});

	// Determine representative for most average during entire run
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		return lhs->GetIsolatedFraction() > rhs->GetIsolatedFraction();
	});*/

	// Populate metadata file
	fMetaData << "Simulation ran for " << timer << "s with population of size " << degrees.size() << std::endl;
	fMetaData << "Omitted " << degrees_omitted << " zeroed degree counts\nOmitted " << degrees_tot_omitted << " zeroed average degree counts" << std::endl;
	fMetaData << "Largest component contained " << components[0].size() << " elements (" << components[0].size() / (double)(degrees.size()) << ")" << std::endl;
	fMetaData << "Highest registered degree count: " << mHighestDegreeCount.value << " from robot " << mHighestDegreeCount.index << " at timeframe " << mHighestDegreeCount.timeFrame << std::endl;
	fMetaData << "Highest registered average degree: " << mHighestAverageDegreeCount.value << " from robot " << mHighestAverageDegreeCount.value <<  " at timeframe " << mHighestAverageDegreeCount.timeFrame << std::endl;
	fMetaData << "Highest registered range: " << mHighestRange.value << " from robot " << mHighestRange.index << " at timeframe " << mHighestRange.timeFrame << std::endl;
	fMetaData << "Highest registered average range: " << mHighestAverageRange.value << " from robot " << mHighestAverageRange.index << " at timeframe " << mHighestAverageRange.timeFrame << std::endl;
	

	fLog << "Done with writing, closing all file handles..." << std::endl;

	// Close file handles
	fDegrees.close();
	fDegreesTot.close();
	fMetaData.close();
	fLog.close();
	fDegreesGC.close();
	fDegreesTotGC.close();
	fDegreesUnsorted.close();
	fDegreesTotUnsorted.close();
	fRanges.close();
	fRangesTot.close();
#endif
	// The file handles should be closed regardless, as they are created at the start
	os_degD.close();
	os_degD_Tot.close();

	//os_interest.close();
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
	int newNode = -1;
	int timeStep = GetSpace().GetSimulationClock();

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
	
	for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
	{
		// Create a pointer to the current foot-bot 
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		std:: string strID = cController.GetId().substr (2,5);
		int unID = std::stoi (strID, nullptr, 10);
		
		// Copy the list of network connections from the data of the range-and-bearing sensor into G
		// Add local links from range-and-bearing sensors			
		CCI_RangeAndBearingSensor::TReadings tPackets = cController.GetTPackets();
		
		if (mOutputInformationAboutZeroWorkAround)
		{	
			if (unID == 0)
			{
				LOG << "Packets: " << cController.GetTPackets().size() << " <> " << tPackets.size() << std::endl;
			}
		}

		if (unID == mNodeOfInterest)
		{
			LOG << "Node: " << unID << " Packets: " << cController.GetTPackets().size() << " <> " << tPackets.size() << std::endl;
		}

		int count = 0;
		for (size_t i = 0; i < tPackets.size(); ++i)
		{
			newNode = tPackets[i].Data[0] * 255 + tPackets[i].Data[1];
			if (newNode == 0)
			{
				count++;
			}
			
			if (std::find(G[unID].begin(), G[unID].end(), newNode) == G[unID].end() 
					&& std::find(G[newNode].begin(), G[newNode].end(), unID) == G[newNode].end())
			{
				// Hacky workaround!!!!!!
				// Ignores populating the connections to the 0th node, as this seems to be some form of unintented behaviour?
				if (mZeroWorkAround)
				{
					if (timeStep == 2 && newNode == 0)
						continue;
				}

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
		
		if (unID == mNodeOfInterest)
		{
			cController.SetInterestNode(true);
			cController.MarkPotentialHub(true);
		}
		
		// --- UPDATE INFORMATION RECEIVED FROM NEIGHBORS ---
		CCI_RangeAndBearingSensor::TReadings newTPackets;
		CCI_RangeAndBearingSensor::SPacket newSPacket;
		
		if (mOutputInformationAboutZeroWorkAround)
		{
			if (unID == 0)
			{
				LOG << "G[unID].size() == " << G[unID].size() << std::endl;
			}
		}

		if (unID == mNodeOfInterest)
		{
			LOG << "G[unID].size() == " << G[unID].size() << std::endl;
		}

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

void CMarchingLoopFunctions::PostStep() 
{
   	Real avgRABRange = 0.0, avgDegree = 0.0;
   	int currentTime = GetSpace().GetSimulationClock();
   	
	// We could speed up the poststep by making these members and resizing them once
	std::vector<int> degDist;
	std::vector<Real> rangeDist;
   	std::vector<CFootBotMarching*> controllers; 
	std::vector<CFootBotEntity*> entities;

   	// Loop over the swarm and record the degree of every robot
   	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	controllers.resize(m_cFootbots.size());
	entities.resize(m_cFootbots.size());
   	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
   	{
		// Create a pointer to the current foot-bot
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
	
		// Get the foot-bot controller
		CFootBotMarching& cController = dynamic_cast<CFootBotMarching&>(cFootBot.GetControllableEntity().GetController());
		std:: string strID = cController.GetId().substr (2,5);
		int unID = std::stoi (strID,nullptr,10);

		// Get the embodied entity as this allows us to access the world position of the actual robot
		// Then set it on the footbot
		CEmbodiedEntity& entity = cFootBot.GetEmbodiedEntity();
		const SAnchor& anchor = entity.GetOriginAnchor();
		cController.SetWorldPosition(anchor.Position);

		entities[unID] = pcFB;

    	avgDegree += cController.GetDegree();
    	avgRABRange += cController.GetNewRABRange();

		const int deg = cController.GetDegree();
		// Update meta data information about the highest degree detected
		if (deg > mHighestDegreeCount.value)
		{
			mHighestDegreeCount.value = deg;
			mHighestDegreeCount.index = unID;
			mHighestDegreeCount.timeFrame = currentTime;
		}
		degDist.push_back(cController.GetDegree());

		// Update meta data information about the highest range detected
		const Real range = cController.GetNewRABRange();
		if (range > mHighestRange.value)
		{
			mHighestRange.value = range;
			mHighestRange.index = unID;
			mHighestRange.timeFrame = currentTime;
		}
		rangeDist.push_back(range);
		
		// Update controllers vector in order to sort by degrees later on
		cController.ResetVisualizationParameters();
		//controllers.push_back(&cController);
		
		controllers[unID] = &cController;
		// TODO: check if this affects the output files! I believe it shouldnt

		/*if (unID == 0)
		{
			LOG << "Current degree count: " << cController.GetDegree() << " Total over time: " << degDistTot[unID] << std::endl;
		}*/

		/*degDistTot[unID] += cController.GetDegree();
		mRangeDistTot[unID] += cController.GetNewRABRange();*/

		//LOG << unID << " ";
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

	Real activeRobots = m_cFootbots.size() * 1.0;

	avgRABRange = avgRABRange / activeRobots;
	avgDegree = avgDegree / activeRobots;

	// Determine the distance to the closest neighbour
	// O(n^2) evaluation, could be simplified with spatial localization
	// Only keep track of square magnitude as this saves overhead
	if (mNNSquaredDistanceDistribution.empty())
		mNNSquaredDistanceDistribution.reserve(controllers.size());

	int entity_i = 0;
	for (CFootBotMarching* footbot : controllers)
	{
		Real nearest = std::numeric_limits<Real>::max();
		CVector3 nearestWorld = CVector3();
		for (CFootBotMarching* other : controllers)
		{
			if (footbot != other) // otherwise nearest will be zero
			{
				Real squareDist = SquareDistance(footbot->GetWorldPosition(), other->GetWorldPosition());
				if (squareDist < nearest)
				{
					nearest = squareDist;
					nearestWorld = other->GetWorldPosition();
				}
			}
		}

		footbot->SetNNSquaredDistance(nearest);
		mNNSquaredDistanceDistribution.push_back(nearest);

		// Draw line between nearest physical neighbour and footbot, cant be done in Qt function due to rotation shenanigans
		// Note that these rays are used by the Qt widget in order to draw communication links
		if (mDrawNNLinks)
		{
			CFootBotEntity* entity = entities[entity_i];
			assert(entity != nullptr);
			CControllableEntity& controllable_entity = entity->GetControllableEntity();
			Real z = 0.05;
			controllable_entity.AddCheckedRay(false, CRay3(footbot->GetWorldPosition() + CVector3(0.0, 0.0, z), nearestWorld + CVector3(0.0, 0.0, z)));

			entity_i++;
		}
	}


	// If the degree of the footbot is in the top x%, then mark it as a potential hub
	// Sort descending by grade (largest to smallest)
	std::sort(controllers.begin(), controllers.end(), [](/*const*/ CFootBotMarching* lhs, CFootBotMarching* rhs)
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
		assert(ptr != nullptr);

		if (counter < (controllers.size() * mHubMarkingFraction))
		{
			ptr->MarkPotentialHub(true);
		}
		counter++;

		// Note, this does the same thing (hopefully) as the sort write code below out of this for loop
		// But will avoid another (expensive) sort call
		degDistTot[controllers.size() - 1 - i] += controllers[i]->GetDegree(); // index 0 gets smallest/last element, index 1 gets ...
		mRangeDistTot[controllers.size() - 1 - i] += controllers[i]->GetNewRABRange();
		i--;
	}

	// TODO: additional sort call, wasteful?
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		if (lhs != nullptr && rhs != nullptr)
		{
			return lhs->GetNewRABRange() > rhs->GetNewRABRange();
		}
		return false;
	});

	counter = 0;
	for (CFootBotMarching* ptr : controllers)
	{
		assert(ptr != nullptr);

		if (counter < (controllers.size() * mHubMarkingFraction))
		{
			ptr->MarkPotentialHighRange(true);
		}
		else if (counter > (controllers.size() - (controllers.size() * mHubMarkingFraction)))
		{
			ptr->MarkPotentialLowRange(true);
		}
			
		counter++;
	}

	// Sort based on nearest neighbour distances
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		return lhs->GetNNSquaredDistance() > rhs->GetNNSquaredDistance();
	});
	counter = 0;
	for (CFootBotMarching* ptr : controllers)
	{
		assert(ptr != nullptr);
		
		/*const int history_degree = ptr->GetLatestDegreeFromHistory();
		if (history_degree != -1)
		{
			assert(history_degree == ptr->GetDegree());
		}*/

		// Separate the robots into different categories depending on the distance 
		// of their nearest neighbours using the fraction parameter mRepresentativeFraction
		CFootBotMarching::EDistanceState state = CFootBotMarching::EDistanceState::INVALID;
		if (counter < (controllers.size() * mRepresentativeFraction))
		{
			state = CFootBotMarching::EDistanceState::ISOLATED;
		}
		else if (counter > controllers.size() - (controllers.size() * mRepresentativeFraction))
		{
			state = CFootBotMarching::EDistanceState::CLUSTERED;
		}
		else
		{
			state = CFootBotMarching::EDistanceState::OTHER;
		}
		ptr->SetDistanceState(state);
		
		counter++;
	}

	// Avoid sorting a second time
	/*std::sort (degDist.begin(), degDist.end());
	for(int i = 0; i < degDist.size(); i++)
   	{
		degDistTot[i] += degDist[i]; 
	}*/
   
	if (currentTime % mOutputTimer == 0)
	{ 
		LOG << GetSpace().GetSimulationClock() << "\t" << "AvgDeg: " << avgDegree << std::endl; 
	}

	if (currentTime == 1)
	{
		OutputNNDistanceDistribution();
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
	// TODO: file init should be better done	
	//os_interest.open("/mnt/c/argos/pl_check_kit/pl_check_kit/nodeinfo.log", std::ofstream::trunc | std::ofstream::out);

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
	mRangeDistTot.resize(m_cFootbots.size());
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
		  mRangeDistTot[unID] = 0.0;
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


void CMarchingLoopFunctions::OutputNNDistanceDistribution()
{
	// Sorted
	std::sort(mNNSquaredDistanceDistribution.begin(), mNNSquaredDistanceDistribution.end());

	std::ofstream fNNDistr;
	fNNDistr.open("/mnt/c/argos/pl_check_kit/pl_check_kit/nnDistribution.dat", std::ofstream::trunc | std::ofstream::out);
	
	for (const Real sqDist : mNNSquaredDistanceDistribution)
	{
		// TODO: what if dist == 0.0?
		fNNDistr << sqrt(sqDist) << std::endl;
	}
	
	fNNDistr.close();
}


REGISTER_LOOP_FUNCTIONS(CMarchingLoopFunctions, "marching_loop_functions")
