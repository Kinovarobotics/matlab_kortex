# MEX interface

## Overview

---

This section gives information on the functions accessible via the KORTEX™ API MEX interface. 

The MEX interface allows you to interact with and control the robot from a MATLAB script. 

MEX interface function calls must be wrapped in a call to `kortexApiMexInterface`, which provides an [entry point](https://www.mathworks.com/help/matlab/apiref/mexfunction.html) to the MEX interface:

`[function_output1, function_output2, ...] = kortexApiMexInterface('function name string', function_input1, ...);`

Examples of use of the MEX interface can be seen in the example `/mex-wrapper/tests_api_control.m`.

Examples can also be seen in the file `kortex.m`. In that file, a `kortex` MATLAB System object uses the KORTEX™ API MEX interface.

Note that all scripts should begin with a call to `CreateRobotApisWrapper`, and end with a call to `DestroyRobotApisWrapper`.

## Functions documentation

---

### Robot initialization, configuration, & de-initialization



####  CreateRobotApisWrapper
This function initializes the KORTEX™ API that operates underneath the MEX interface.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| char* | The IP address of the robot. |
| char* | The username that will initiate a session on the robot. |
| char* | The password linked to the username. |
| uint32_t | The session timeout. |
| uint32_t | The control timeout. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| uint32_t | An handle to the API. this handle will be needed by most of the function of this MEX interface.|
| uint32_t | The device ID of the vision module attached to the robot. |

####  DestroyRobotApisWrapper
This function closes the connection with the KORTEX™ API cleanly.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |


####  SetAdmittance
Set the robot into admittance mode.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The desired [AdmittanceMode](data_structure.md#admittancemode). |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  SetServoingMode
Set the robot servoing mode.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The desired [ServoingMode](data_structure.md#servoingmode). |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success(true) or not(false). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

---

### Emergency, troubleshooting, fault handling

####  ApplyEmergencyStop
Apply the emergency stop on the robot. The robot should stop moving and go to the fault state.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  ClearFaults
Try to clear the faults on the robot. If the robot is in a fault state, you should see a red LED on the robot's base.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |


####  Reboot
Reboot the robot. Warning: the robot may fall slowly for a short time because during the reboot process, the joints stop the servoing process.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |


####  StopAction
Stop the current trajectory.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  PauseAction
Pause the current trajectory.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  ResumeAction
Resume the current trajectory.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

---

### Robot information, status, and feedback


####  GetJointCount
Returns the joint count of the robot.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | The handle of the API. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| uint32_t | The joint count. |


####  GetMovementStatus
Returns the status of the robot.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| int32_t | Status of the robot. This value is described [here](data_structure.md#movementstatus). |


####  RefreshFeedback
It returns the feedback data from the base, the joints and tool.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type     |           Description           |
|:-------------:|:-------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| [BaseFeedback](data_structure.md#basefeedback) | Feedback data for the base.     |
| [ActuatorsFeedback](data_structure.md#actuatorsfeedback) | Feedback data for each joint. |
| [InterconnectFeedback](data_structure.md#interconnectfeedback) | Feedback data for the interface module, including gripper.    |

---

### Robot high-level control


####  ReachCartesianPose
Move the robot to a specific Cartesian pose. The orientation is represented with Euler angles and the convention used is the **Tait-Bryan, extrinsic ZYX**

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The [CartesianConstraintType](data_structure.md#cartesianconstrainttype) applied on the movement. |
| double\[2\] | Speed limitation that will be applied on the trajectory if the constraint has been set to CARTESIAN_CONSTRAINT_SPEED. The first value is the translation velocity and the second value is not used for now. |
| double | Not supported for now. |
| double\[3\] | Translation (X, Y, Z) in meters. |
| double\[3\] | Orientation. (X(gamma), Y(beta), Z(alpha)) in degrees. (**Tait-Bryan, extrinsic ZYX**) |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  ReachJointAngles
Move the robot to a specific joint pose.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The [JointConstraintType](data_structure.md#jointconstrainttype) applied on the movement. |
| double | Speed limitation, in degrees / second,  that will be applied on the trajectory if the constrait has been set to JOINT_CONSTRAINT_SPEED. |
| double | Duration of the trajectory that will be applied on the trajectory if the constraint has been set to JOINT_CONSTRAINT_DURATION. |
| double\[7\] | An angle, in degrees, for each joint of the robot. The first index is the nearest joint to the base and the last index is the nearest joint to the gripper. |


##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  SendJointSpeedCommand
Move the robot using a speed command.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| double | Duration, in seconds, of the speed command. |
| double\[7\] | An speed command, in degrees / second, for each joint of the robot. The first index is nearest joint to the base and the last index is the nearest joint to the gripper. |
| uint32_t | Joint count. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  SendToolCommand
Move the robot tool. For now, our gripper is considered as one tool so only one command (first index) is needed to move the tool.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The [mode](data_structure.md#toolmode) used to move the tool. |
| double | Duration constraint, in sec, of the tool command. If set to 0 then no cosntraint is applied. |
| double\[10\] | A command for each possible tool. Each command is a value between 0 and 1. 0 means fully opened and 1 means fully closed. |
| uint32_t | Tool count. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |


####  PlayPreComputedTrajectory
Send a Precomputed Joint Trajectory to the robot. A Precomputed Joint Trajectory is a set of (timestamp, angular position, angular velocity, angular acceleration) for each joint at each increment of the trajectory. Together, this represents a trajectory. For more information on the Precomputed Joint Trajectories, you can read [the documentation on Precomputed Joint Trajectories](./precomputed_joint_trajectories.md).

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The desired [TrajectoryContinuityMode](data_structure.md#trajectorycontinuitymode). |
| double\[7\]\[30000\] | A matrix that contains a list of angular positions (degrees) for each joint at each increment. |
| double\[7\]\[30000\] | A matrix that contains a list of angular velocities (degrees / second) for each joint at each increment. |
| double\[7\]\[30000\] | A matrix that contains a list of angular accelerations (degrees / second ^ squared) for each joint at each increment. |
| double\[7\]\[30000\] | A matrix that contains a list of timestamps for each joint at each increment. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

--- 

### Vision module configuration



####  SetSensorSettings
Configure one of the sensors (either 2D or 3D) of the vision module. 

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The sensor ID you want to configure. This value is described [here](data_structure.md#sensor). |
| int32_t | The desired resolution. See [Resolution](data_structure.md#resolution). |
| int32_t | The desired frame rate. See [FrameRate](data_structure.md#framerate). |
| int32_t | The desired bit rate. See [BitRate](data_structure.md#bitrate). |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  SetOptionValue
Configure a specific option out of the options available on the vision module. 

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| int32_t | The sensor ID you want to configure. See [Sensor](data_structure.md#sensor). |
| int32_t | The desired option. See [Option](data_structure.md#option). |
| double | The desired value of the specified option. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  InitVision
Initialize the vision module API. 

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | The handle of the API. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |

####  GetSensorSettings
Get the configuration of a specific sensor(2D or 3D) of the vision module.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| uint32_t | ID of the desired sensor. See [Sensor](data_structure.md#sensor).|

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| uint32_t | The resolution of the sensor. See [Resolution](data_structure.md#resolution). |
| uint32_t | The frame rate of the sensor. See [FrameRate](data_structure.md#framerate). |
| uint32_t | The bit rate of the sensor. See [BitRate](data_structure.md#bitrate). |

####  GetOptionValue
Get the value of a specific option from a specific sensor (eitrher 2D or 3D) of the vision module.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| uint32_t | The desired sensor. See [Sensor](data_structure.md#sensor).|
| uint32_t | The desired option. See [Option](data_structure.md#option).|

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| uint32_t | The resolution of the sensor. This value is described [here](data_structure.md#resolution). |
| uint32_t | The frame rate of the sensor. This value is described [here](data_structure.md#framerate). |
| uint32_t | The bit rate of the sensor. This value is described [here](data_structure.md#bitrate). |

####  GetIntrinsicParameters
Get the intrinsic parameters from a specific sensor (either 2D or 3D) of the vision module.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| uint32_t | The desired sensor. This value is described [here](data_structure.md#sensor).|

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success. |
| [IntrinsicParameters](data_structure.md#intrinsicparameters) | Sensor intrinsic parameters. |

####  GetExtrinsicParameters
Get the extrinsic parameters of the vision module.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle |


##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success. |
| [ExtrinsicParameters](data_structure.md#extrinsicparameters) | Sensor extrinsic parameters |

---

### Errors information


####  GetLastError
Returns the last error that occurred in the API.

##### Input

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| uint32_t | API Handle. |

##### Output

|      C++ Type      |                          Description                          |
|:--------------:|:-------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| uint32_t | The error code of the last error triggered by the Kortex API. |

####  GetErrorName
Returns a string that describe the error code provided as an input.

##### Input

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| uint32_t | API Handle. |
| uint32_t | The error code that you want a name for. |

##### Output

|      C++ Type     |                                                    Description                                                   |
|:-------------:|:----------------------------------------------------------------------------------------------------------------:|
| bool | A flag that tells if the operation was a success (`true`) or not (`false`). |
| [ErrorStructMatrix](data_structure.md#errorstructmatrix) | A data structure that contains the error code if the operation was not a success.  |
| char * | The name that describe the error code. |


