# Marauders Map

[![Build Status](https://travis-ci.org/karanvivekbhargava/marauders_map.svg?branch=master)](https://travis-ci.org/karanvivekbhargava/marauders_map)

Marauders Map is a turtlebot package which will use a mapping/slam package and exploratory behaviour to map indoor environments.

## Build Instructions

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/karanvivekbhargava/marauders_map.git
cd ..
catkin_make
```

## Running rostest
The unit tests have been written using gtest and rostest. To run the tests, you need to be in the catkin workspace parent folder. Then run the commands below

```
cd <path to catkin workspace>
catkin_make run_tests
```
You can test using
```
rostest marauders_map pathPlannerTest.launch
```

## SIP process
Log details for the same can be found [here](https://docs.google.com/spreadsheets/d/1UN-LUKyeZunZTpRnJA9aYaXh8SntVCdyPhzmPg2l0AY/edit#gid=0)

Planning notes can be found [here](https://docs.google.com/document/d/1BU2oDnlLBrMnNgZKm1wX3ZKhkwUWr54Su3-iXXmer3M/edit?usp=sharing)

## Tasks done
* Created repository with license, package.xml and CMakelists
* Setup travis ci
* Created UML diagrams
* Explored various packages to find a good octree map of the environment.
* Implemented stubs for classes
* Implemented unit tests for the classes
* Implemented classes to pass the tests (test-based development)
