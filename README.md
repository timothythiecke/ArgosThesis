# ArgosThesis

Repository created to keep track of my master thesis during the period of june 2019 - january 2020.

## Installation
In order to run the experiment, you first need to install ARGoS on your computer or VM. Follow the installation guide on [the ARGoS Github](https://github.com/ilpincy/argos3 "ARGoS"). Before you continue, make sure to copy the two files from [the space_diff folder](https://github.com/timothythiecke/ArgosThesis/tree/master/space_diff) to your ARGoS installation folder `argos3\src\core\simulator\space` and make ARGoS again (these are required for the ring distributor of the experiment).


Then, create a subfolder in your main ARGoS folder and clone this repository into it. Create a build directory in the cloned repo. The installation is similar to the one from the [ARGoS examples Github](https://github.com/ilpincy/argos3-examples "ARGoS examples"). After installation, from the root folder of the cloned repo, run `argos3 -c experiments/marching_large.argos` to start the simulator.


## Datasets
For verification, you can check the [datasets folder](https://github.com/timothythiecke/ArgosThesis/tree/master/datasets). Using the scripts from the [scripts folder](https://github.com/timothythiecke/ArgosThesis/tree/master/scripts) allows you to verify the generated data.

## TODO list
* The loopfunction requires cleanup of the way file handles are used
