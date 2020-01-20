# ArgosThesis

Repository created to keep track of my master thesis during the period of june 2019 - january 2020.

## Installation
In order to run the experiment, you first need to install ARGoS on your computer or VM. Follow the installation guide on [the ARGoS Github](https://github.com/ilpincy/argos3 "ARGoS"). Before you continue, make sure to copy the two files from [the space_diff folder](https://github.com/timothythiecke/ArgosThesis/tree/master/space_diff) to your ARGoS installation folder `argos3\src\core\simulator\space` and make ARGoS again (these are required for the ring distributor of the experiment).


Then, create a subfolder in your main ARGoS folder and clone this repository into it. Create a build directory in the cloned repo. The installation is similar to the one from the [ARGoS examples Github](https://github.com/ilpincy/argos3-examples "ARGoS examples"). After installation, from the root folder of the cloned repo, run `argos3 -c experiments/marching_large.argos` to start the simulator.


## Datasets
For verification, you can check the [datasets folder](https://github.com/timothythiecke/ArgosThesis/tree/master/datasets). Using the scripts from the [scripts folder](https://github.com/timothythiecke/ArgosThesis/tree/master/scripts) allows you to verify the generated data in comparison with the tables and data presented in the thesis.

### The DMMAnalysis files
Each file follows the following format: `x_y_z_w_type` where x stands for seed, y for the population size, z for t_end, and type for which type of distribution is contained in the file. w differs: 00 stands for DMM disabled, 01 stands for local neighbourhood check enabled, 10 stands for breakdown check enabled and 11 for DMM fully enabled. Some files have sanitized.dat behind them, indicating that zeroed values were omitted.

### The GeneralPowerlaw files
Contains seed averaged degree distributions (S=30)

### The HeuristicData files
Contains files following `seed_population_tend_heuristic_type_datatype` files. Defined heuristics: extreme, random, postdegree. Types: min, mid, max. Datatypes equivalent to which distribution is contained.

### Longrun files
Contains the files associated with the longrun of the experiment. The .lr files are basic kappa and lambda distribution files. Degrees and ranges.csv contain the min, median, max, and average values per timestep. Meta.lr contains information containing the DFS search of the components of the global graph G.

### PL evaluator files
Contains a subfolder with seeddata in the same vein as the DMMAnalysis files without the w component in the filename. The result of recurring PL evaluator tests (binomial) are then processed and formatted into the .csv files.

## TODO list
* Readme: explain what each .argos file configurable variable does
* The Loopfunction requires cleanup of the way file handles are used
* Cleanup sorting in Destroy() should be reduced as much as possible
* Naive nearest neighbour evaluation should be replaced with a more efficient technique
* Controller class knows of the state of a lot of other robots as well as their history data, it would be cleaner if this data is stored in the Loopfunction as this better follows the design pattern
* Similarly, a node/controller marked for interest handles a file handle creation and deletion, this should also be handled by the loopfunction

