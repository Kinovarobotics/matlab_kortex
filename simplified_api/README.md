# KINOVA<sup>®</sup> KORTEX™ MATLAB API

## Overview
This MATLAB adaptor allows communication via MATLAB with a KINOVA<sup>®</sup> <i>Gen3 Ultra lightweight robot</i>.

## System requirements
This software has been tested on **Ubuntu 16.04 (64-bit)** and **Windows 10** 

### MATLAB
1. [MATLAB](https://www.mathworks.com/products/matlab.html) version 9.8 (R2020a)
2. Simulink Coder Toolbox
3. Simulink Toolbox
4. Simscape Multibody Toolbox
5. Simscape Toolbox
6. Robotics System Toolbox
7. Optimization Toolbox
8. MATLAB Coder Toolbox
9. Embedded Coder Toolbox

## Download instructions

## Installation Instructions
You can access the latest release of the MATLAB adaptor [here](https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public%2Fkortex%2Fmatlab%2Fsimplified_API%2F2.2.1%2Fmatlab_simplified_api_2.2.1.zip) on the Kinova Artifactory.
( The release contains compiled libraries for Linux x86 architecture and Windows Visual Studio 2015 )

### Windows Operating System
From the Kinova Artifactory:
* Expand `kortex/matlab/simplified_api` to see the release folders
* Right-click on the desired release folder
* Select `Download` in the context menu
* Choose the desired Archive Type
* Click on `Download`
* Extract the MATLAB adaptor folders from the downloaded archive.

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

Refer to the dedicated page for [Setup](documentation/setup.md)

## Usage

There are two ways to interact with the robot via MATLAB.

1. Using the KORTEX™ API MEX interface (`kortexApiMexInterface.cpp`)
2. Using the MATLAB System object `kortex` (which itself uses the KORTEX™ API MEX interface)

The KORTEX™ API MEX interface is the best option for controlling the arm via a MATLAB script.
The `kortex` System object on the other hand is intended for dynamical simulations of the robot as part of a system in [Simulink](https://www.mathworks.com/help/simulink/define-new-system-objects.html).

For more information on both options:

* [MEX interface](documentation/mex_interface.md)
* [System object](documentation/system_object.md)

In addition, MATLAB Code Generation support for generating a ROS2 node and deploying the `kortex` System Object node directly on a NVidia Jetson Xavier platform is available. For more information, refer to the [ROS2 Code Generation on Jetson Target](documentation/setup.md#ROS2-Code-Generation-on-Jetson-Target) section in the Setup page.

For more information on MATLAB code generation, see this [Mathworks documentation page](https://www.mathworks.com/help/mpc/code-generation.html).
