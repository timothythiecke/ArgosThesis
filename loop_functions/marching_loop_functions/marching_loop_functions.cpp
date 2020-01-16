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
		GetNodeAttribute(tRepresentative, "heuristic", mRepresentativeHeuristicFromFile);
		GetNodeAttribute(tRepresentative, "maxNode", mRepresentativeOfMaxID);
		GetNodeAttribute(tRepresentative, "midNode", mRepresentativeOfMidID);
		GetNodeAttribute(tRepresentative, "minNode", mRepresentativeOfMinID);

		mRepresentativeHeuristic = (ERepresentativeHeuristic)mRepresentativeHeuristicFromFile;
		LOG << "Using heursitic " << (int)mRepresentativeHeuristic << std::endl;

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
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

	timer = GetSpace().GetSimulationClock();
	int seed = CSimulator::GetInstance().GetRandomSeed();
	int pop_size = m_cFootbots.size();

	//OutputSeedData();

#define DIRECT_OUTPUT // If defined, outputs files directly to the pl_check_kit folder
#ifdef DIRECT_OUTPUT
	// Create file handles
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

	vector<std::ofstream*> file_handles = { &fDegrees, &fDegreesTot, &fMetaData, &fLog, &fDegreesGC, &fDegreesTotGC, &fDegreesUnsorted, &fDegreesTotUnsorted, &fRanges, &fRangesTot};
	std::string dir = "/mnt/c/argos/pl_check_kit/pl_check_kit/";
	vector<std::string> names = { "degDistribution.dat", "degDistribution_tot.dat", "metaData.dat", "log.log", "degDistributionGiant.dat", "degDistribution_totGiant.dat",
									"degDistributionUnsorted.dat", "degDistribution_totUnsorted.dat","ranges.dat", "ranges_tot.dat" };
	
	int i = 0;
	for (std::ofstream* file : file_handles)
		file->open(dir + names[i++]);

	// File handles for seed data
	CFootBotEntity& bot = *any_cast<CFootBotEntity*>(m_cFootbots.begin()->second);
	CFootBotMarching& controller = dynamic_cast<CFootBotMarching&>(bot.GetControllableEntity().GetController());
	i = 0;
	dir = "/mnt/c/argos/pl_check_kit/pl_check_kit/SeedData/";
	vector<std::ofstream> seed_based_files(7);
	vector<std::string> seed_based_names = { "deg", "degTime", "NN", "NNTime", "range", "rangeTime", "meta"};
	assert(seed_based_files.size() == seed_based_names.size());
	for (std::ofstream& file : seed_based_files)
	{
		int breakdown = controller.GetBreakdownEnabled() ? 1 : 0;
		int local = controller.GetLocalNeighbourhoodCheck() ? 1 : 0;
		file.open(dir + std::to_string(seed) + "_" + std::to_string(pop_size) + "_" + std::to_string(timer)+ "_" + std::to_string(breakdown) + std::to_string(local) + "_" + seed_based_names[i++]);
	}

	std::vector<int> degrees(pop_size);
   	std::vector<Real> degrees_tot(pop_size);

	std::vector<Real> ranges(pop_size);
	std::vector<Real> ranges_tot(pop_size);

	std::vector<Real> nn(pop_size);
	std::vector<Real> nn_tot(pop_size);
#endif
	// Export the degree distribution from degDistTot
	std::vector<CFootBotMarching*> controllers(pop_size);
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

		nn[unID] = cController.GetNNSquaredDistance();
		nn_tot[unID] = mNNDistanceDistTot[unID] / (Real)(timer);
#else
		os_degD << cController.GetDegree() << std::endl;
		os_degD_Tot << degDistTot[unID]*1.0/(timer*1.0) << std::endl;
#endif

		cController.CalculateHistoryAverages();
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
		return lhs->GetAverageNNDistanceOverTime() > rhs->GetAverageNNDistanceOverTime();
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
	isolatedMeta << controllers[0]->GetID() << std::endl;
	for (CFootBotMarching* ptr : controllers)
	{
		isolatedMeta << ptr->GetAverageNNDistanceOverTime() << std::endl;
	}
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


	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		return lhs->GetNewRABRange() > rhs->GetNewRABRange();
	});

	std::ofstream shiefefl;
	std::string filename;
	filename.append("/mnt/c/argos/pl_check_kit/pl_check_kit/rangedeg").append(std::to_string(controllers.size())).append(".dat");
	//shiefefl.open("/mnt/c/argos/pl_check_kit/pl_check_kit/rangedeg.dat");
	shiefefl.open(filename);
	for (CFootBotMarching* ptr : controllers)
	{
		assert(ptr != nullptr);

		if (ptr->GetDegree() > 0)
			shiefefl << ptr->GetNewRABRange() << "," << ptr->GetDegree() << std::endl;
	}
	shiefefl.close();
	
	this->OutputDataForHeuristic(controllers);

	filename.clear();
	filename.append("/mnt/c/argos/pl_check_kit/pl_check_kit/degrange").append(std::to_string(controllers.size())).append(".dat");
	//shiefefl.open("/mnt/c/argos/pl_check_kit/pl_check_kit/degrange.dat");
	shiefefl.open(filename);
	for (CFootBotMarching* ptr : controllers)
	{
		assert(ptr != nullptr);
		if (ptr->GetDegree() > 0)
			shiefefl << ptr->GetDegree() << "," << ptr->GetNewRABRange() << std::endl;
	}
	shiefefl.close();


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
	for (std::ofstream* f : file_handles)
		f->close();

	std::sort(nn.begin(), nn.end(), [](const double i, const double j)
	{
		return i < j;
	});

	std::sort(nn_tot.begin(), nn_tot.end(), [](const double i, const double j)
	{
		return i < j;
	});

	std::sort(ranges.begin(), ranges.end(), [](const double i, const double j)
	{
		return i < j;
	});

	std::sort(ranges_tot.begin(), ranges_tot.end(), [](const double i, const double j)
	{
		return i < j;
	});

	for (int i = 0; i < m_cFootbots.size(); i++)
	{
		//if (degrees[i] > 0)
		seed_based_files[0] << degrees[i] << std::endl;
		
		//if (degrees_tot[i] > 0)
			seed_based_files[1] << degrees_tot[i] << std::endl;

		//if (nn[i] > 0)
			seed_based_files[2] << sqrt(nn[i]) << std::endl;
		
		//if (nn_tot[i] > 0)
			seed_based_files[3] << sqrt(nn_tot[i]) << std::endl;

		///if (ranges[i] > 0)
			seed_based_files[4] << ranges[i] << std::endl;

		//if (ranges_tot[i] > 0)
			seed_based_files[5] << ranges_tot[i] << std::endl;
	}

	seed_based_files[6] << mAvgDegree << "," << mAvgNNDistance << "," << mAvgRABRange << std::endl;
   
	// Close other file handles
	for (std::ofstream& file : seed_based_files)
		file.close();
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
	// Unsure about resetting here
	mAvgRABRange = 0.0;
	mAvgDegree = 0.0;
	mAvgNNDistance = 0.0;
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

    	mAvgDegree += cController.GetDegree();
    	mAvgRABRange += cController.GetNewRABRange();
		mAvgNNDistance += cController.GetNNSquaredDistance();

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

	mAvgRABRange /= (Real)(m_cFootbots.size());
	mAvgDegree /= (Real)(m_cFootbots.size());
	mAvgNNDistance /= (Real)(m_cFootbots.size());

	// Determine the distance to the closest neighbour
	// O(n^2) evaluation, could be simplified with spatial localization
	// Only keep track of square magnitude as this saves overhead
	if (mNNSquaredDistanceDistribution.empty())
		mNNSquaredDistanceDistribution.reserve(controllers.size());

	int entity_i = 0;
	for (CFootBotMarching* footbot : controllers)
	{
		Real nearest = std::numeric_limits<Real>::max();
		Real range = -1.0;
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
					range = other->GetNewRABRange();
				}
			}
		}

		footbot->SetNNSquaredDistance(nearest);
		footbot->SetNNRange(range); // For history data
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
	
	i = controllers.size() - 1;
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

		mRangeDistTot[controllers.size() - 1 - i] += controllers[i]->GetNewRABRange();

		i--;
		counter++;
	}

	// Steup heuristic
	if (currentTime == 1 && mRepresentativeHeuristic != ERepresentativeHeuristic::Invalid)
	{
		LOG << "Setting up heuristic..." << std::endl;
		this->SetupHeuristic(controllers);
	}

	// Avoid sorting a second time
	/*std::sort (degDist.begin(), degDist.end());
	for(int i = 0; i < degDist.size(); i++)
   	{
		degDistTot[i] += degDist[i]; 
	}*/
   
	if (currentTime % mOutputTimer == 0)
	{ 
		LOG << GetSpace().GetSimulationClock() << "  " << "AvgDeg: " << mAvgDegree << std::endl; 
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
    int pop_size = m_cFootbots.size();
	degDistTot.resize(pop_size);
	mRangeDistTot.resize(pop_size);
	mNNDistanceDistTot.resize(pop_size);
    G.resize(pop_size);
    
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


// Other function as the destroy function is all over the place at the moment
void CMarchingLoopFunctions::OutputSeedData()
{
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

	timer = GetSpace().GetSimulationClock();
	int seed = CSimulator::GetInstance().GetRandomSeed();
	int pop_size = m_cFootbots.size();
}


// Precondition: all indices are valid
void CMarchingLoopFunctions::SetupHeuristic(std::vector<CFootBotMarching*>& controllers)
{
	// Sort based on nearest neighbour distances
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		return lhs->GetNNSquaredDistance() > rhs->GetNNSquaredDistance();
	});

	// Do the following unless the representative ID variables have been filled in already
	// Note then that this does not use a heuristic
	if (mRepresentativeOfMaxID != -1 || mRepresentativeOfMidID != -1 || mRepresentativeOfMinID != -1)
	{
		return;
	}

	if (mRepresentativeHeuristic == ERepresentativeHeuristic::SortedExtremes)
	{
		// Select and mark the following nodes
		// -> 0 is marked for top
		// -> controllers.size() -1 is marked for low
		// -> controllers.size() / 2 is marked for mid
		mRepresentativeOfMaxID = controllers[0]->GetID();
		mRepresentativeOfMinID = controllers[controllers.size() - 1]->GetID();
		mRepresentativeOfMidID = controllers[controllers.size() / 2]->GetID();
		
		LOG << "Heuristic succesfully determined indices (extremes): " << mRepresentativeOfMaxID << " " << mRepresentativeOfMinID << " " << mRepresentativeOfMidID << std::endl;
	}
	else if (mRepresentativeHeuristic == ERepresentativeHeuristic::SortedRandomChoice)
	{
		// Select and mark the following nodes
		// -> randomly select out of top fraction and mark for top
		// -> randomly select out of low fraction and mark for low
		// -> randomly select out of mid section and mark for mid
		int top_l = 0;
		int top_r = (int)(controllers.size() * mRepresentativeFraction);
		int low_l = controllers.size() - top_r;
		int low_r = controllers.size() - 1;
		
		int mid_l = top_r + 1;
		int mid_r = low_r - 1;
		
		mRepresentativeOfMaxID = controllers[m_pcRNG->Uniform(CRange<UInt32>(top_l, top_r))]->GetID();
		mRepresentativeOfMinID = controllers[m_pcRNG->Uniform(CRange<UInt32>(low_l, low_r))]->GetID();
		mRepresentativeOfMidID = controllers[m_pcRNG->Uniform(CRange<UInt32>(mid_l, mid_r))]->GetID();

		LOG << "Heuristic succesfully determined indices (random): " << mRepresentativeOfMaxID << " " << mRepresentativeOfMinID << " " << mRepresentativeOfMidID << std::endl;
	}
	else if (mRepresentativeHeuristic == ERepresentativeHeuristic::PostExperimentDegreeCount)
	{
		// Try and open the file which contains the three indices of top, low, and mid based on degree counts at the end of a previous simulation run
		// File is outputted at the end of simulation in Destroy()
		// If it doesn't exist, then mRepresentativeHeuristic is reinitialized
		// otherwise mark the nodes for top, low and mid respectively
		std::ifstream indexFile;
		indexFile.open(mHeuristicFileName);
		if (indexFile.is_open())
		{
			indexFile >> mRepresentativeOfMaxID >> mRepresentativeOfMinID >> mRepresentativeOfMidID;

			LOG << "Heuristic succesfully determined indices (postdegree): " << mRepresentativeOfMaxID << " " << mRepresentativeOfMinID << " " << mRepresentativeOfMidID << std::endl;

			indexFile.close();
		}
		else
		{
			mRepresentativeHeuristic = ERepresentativeHeuristic::Invalid;
			LOG << "Could not find index file for heuristic!" << std::endl;
		}
	} 	// Largely dependent on t_end!
}



void CMarchingLoopFunctions::OutputDataForHeuristic(std::vector<CFootBotMarching*>& controllers)
{
	std::sort(controllers.begin(), controllers.end(), [](CFootBotMarching* lhs, CFootBotMarching* rhs)
	{
		assert(lhs != nullptr);
		assert(rhs != nullptr);

		return lhs->GetDegree() > rhs->GetDegree();
	});
	// Output the indices of the node with the most amount of degrees, smallest and middle point
	if (GetSpace().GetSimulationClock() == 100)
	{
		std::ofstream heuristicIndexFile;
		heuristicIndexFile.open(mHeuristicFileName);
		heuristicIndexFile << controllers[0]->GetID() << "\n" << controllers[controllers.size() - 1]->GetID() << "\n" << controllers[controllers.size() / 2]->GetID() << std::endl;
		heuristicIndexFile.close();
	}

	if (mRepresentativeHeuristic == ERepresentativeHeuristic::Invalid)
		return;
	
	std::vector<std::string> filenames = { "range", "nnrange", "nndist", "directiondecision", "degree" };
	std::string folder = "/mnt/c/argos/pl_check_kit/pl_check_kit/HeuristicData/";
	int seed = CSimulator::GetInstance().GetRandomSeed();
	int size = controllers.size();
	int time = GetSpace().GetSimulationClock();
	std::vector<std::string> heuristicNames = { "extreme", "random", "postdegree"};
	std::vector<std::string> type = { "max", "min", "mid" };

	// folder/seed_swarmsize_time_heuristic_type_datatype.dat
	// folder/seed_heuristic_type_datatype
	std::vector<int> indices = { mRepresentativeOfMaxID, mRepresentativeOfMinID, mRepresentativeOfMidID };
	int i = 0;
	for (const int index : indices)
	{
		std::vector<CFootBotMarching*>::iterator it = std::find_if(controllers.begin(), controllers.end(), [&index](const CFootBotMarching* ptr)
		{	
			assert(ptr != nulltptr);
			return ptr->GetID() == index;
		});

		if (it != controllers.end())
		{
			std::ofstream oRange;
			std::ofstream oNNRange;
			std::ofstream oNNDistance;
			std::ofstream oDirectionDecision;
			std::ofstream oDegree;

			std::string base = folder + std::to_string(seed) + "_" + std::to_string(size) + "_" + std::to_string(time) + "_" + heuristicNames[(int)(mRepresentativeHeuristic)] + "_" + type[i] + "_";
			oRange.open(base + filenames[0]);
			oNNRange.open(base + filenames[1]);
			oNNDistance.open(base + filenames[2]);
			oDirectionDecision.open(base + filenames[3]);
			oDegree.open(base + filenames[4]);

			const vector<CFootBotMarching::SHistoryData>& history = (*it)->GetHistory();
			int d = 0;
			for (int c = 0; c < history.size(); c++)
			{
				oRange << history[c].Range << std::endl;		
				if (c == 0) // Edge case hack, use the nearest neighbour distance of the first nonzero element
					d = 1;
				else
				d = c;
				oNNDistance << sqrt(history[d].NearestNeighbourDistance) << std::endl;
				oNNRange << history[d].NearestNeighbourRange << std::endl;
				oDirectionDecision << history[c].DirectionDecision << std::endl;
				oDegree << history[c].Degree << std::endl;
			}

			oRange.close();
			oNNRange.close();
			oNNDistance.close();
			oDirectionDecision.close();
			oDegree.close();
		}
		else
		{
			throw "Index could not be found in controllers vector";
		}
		i++;
	}
}



REGISTER_LOOP_FUNCTIONS(CMarchingLoopFunctions, "marching_loop_functions")
