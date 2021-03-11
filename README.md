# RRT
This is a basic implementation of RRT on a 2D world with line and circle obstacles

To use the application, unpack the provided zip file and build and run using the instructions below.

## Dependencies

The application has dependencies that will be available with out of the box Ubuntu 20.04 operating system. They are as follows,

* *Operating system*: Ubuntu 20.04 or equivalent
* *CMake* 3.8.10 or above
* *gcc/g++* 7.5.0 or above

## Build

The application builds using **CMake** 

Within the root directory of the application, the source files are in *./src* directory and the included headers are in *./include*. Two scripts *build.sh* and *run.sh* are provided for building and running the application.

Open a terminal window and follow the steps below,

* Make sure the scripts have execute permissions by running the command `ls -l`
* Run the provided build script using the command `./build.sh`

## Run

Open a new terminal window if not following from the "Build" section and execute the following commands from within the root directory of the application,

* Run the provided run script using the command `./run.sh`

## Config file
Configure the RRT and other world parameters in the config file `config/rrt_config.yaml`
