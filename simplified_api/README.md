# KINOVA<sup>®</sup> KORTEX™ MATLAB API

## Overview
This MATLAB adaptor allows communication via MATLAB with a KINOVA<sup>®</sup> <i>Gen3 Ultra lightweight robot</i>.

## System requirements
This software has been tested on **Ubuntu 16.04 (64-bit)** and **Windows 10** 

### MATLAB
1. [MATLAB](https://www.mathworks.com/products/matlab.html) version 9.5 (R2018b)
2. Simulink Coder Toolbox
3. Simulink Toolbox
4. Simscape Multibody Toolbox
5. Simscape Toolbox
6. Robotics System Toolbox
7. Optimization Toolbox
8. MATLAB Coder Toolbox
9. Embedded Coder Toolbox

## Download instructions

You can download the latest firmware package for the Gen3 from [here](https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/gen3/2.0.0/Gen3-2.0.0.swu).

You can download the latest Gen3 release notes from [here](https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/gen3/2.0.0/RN-001_KINOVA_Gen3_Ultra_lightweight_robot-Release_Notes_EN_R04.pdf).

## Installation Instructions
You can access the latest release of the MATLAB adaptor [here](https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/matlab/simplified_API/2.0.0/matlab_simplified_api_2.0.0.zip) on the Kinova Artifactory.

Uncompress the MATLAB adaptor folders from the downloaded archive.

## Build instructions

The compiled MEX files are already present in the package : kortexApiMexInterface.mexa64 (Ubuntu 16.04 64 bits) and kortexApiMexInterface.mexw64 (Windows 10 64 bits).
However, you can still build those files yourself following these instructions:

### Build Environment
* CMake version 3.10 or more recent

#### Windows Operating System
* [Build Tools for Visual Studio 2017](https://visualstudio.microsoft.com/vs/older-downloads/)
* Windows Command Prompt

#### Linux Operating System
* GCC/G++
* Pkg-config
```sh
sudo apt install build-essential pkg-config
```

### How to build
- Open MATLAB and navigate to the `mex-wrapper` folder.
- Add the current folder and all of its subfolders to the MATLAB path.
- Run the script `compileKortexMex.m` and wait until its completion.

You are now ready to use the KORTEX™ MEX (MATLAB executable) interface.

## Usage

There are two ways to interact with the robot via MATLAB.

1. Using the KORTEX™ API MEX interface (`kortexApiMexInterface.cpp`)
2. Using the MATLAB System object `kortex` (which itself uses the KORTEX™ API MEX interface)

The KORTEX™ API MEX interface is the best option for controlling the arm via a MATLAB script.
The `kortex` System object on the other hand is intended for dynamical simulations of the robot as part of a system in [Simulink](https://www.mathworks.com/help/simulink/define-new-system-objects.html). 

For more information on both options:

* [MEX interface](documentation/mex_interface.md)
* [System object](documentation/system_object.md)


