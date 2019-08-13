# kortex System object

## Overview

---

The file `kortex.m` defines a MATLAB [System object](https://www.mathworks.com/help/matlab/matlab_prog/define-basic-system-objects-1.html) class `kortex` that lets you use some of the functions available in the KORTEX™ API [MEX interface](mex_interface.md). 

The `kortex` System object represents the KINOVA<sup>®</sup> Gen3 <i>Ultra lightweight robot</i> as a dynamical system component 'black box' with inputs and outputs.

An example of such a model is provided in `mex-wrapper/example_model.slx`.

## Object interface description 
---

The following describes the inputs and outputs of the `kortex` System object.


### Inputs


#### cmdToProcess
The ID of the command you want to process. It is defined by the the ENUM enumCmd located in the folder `mex-wrapper/enums/enumCmd.m`.

| Enum | Value | Description |
|:---------------------:|:------:|-----------------------------------------------------|
| undefined | 0 |You should not use this value. For internal use only. |
| reboot_arm | 1 |Initiate a software reboot on the arm. |
| emergency_stop | 2 |Activate the emergency stop. |
| clear_faults | 3 |Attempt to clear the faults on the robot. |
| stop_action | 4 |Stop the current action |
| pause_action |  5 | Pause the current action |
| resume_action | 6 | Resume the current action |
| precomputed_joint_trj | 101 | Send a pre computed trajectory to the robot. |
| joint_reach | 200 | Send a joint pose to the robot. |
| cartesian_reach | 300 | Send a Cartesian position to the robot. |
| tool_reach | 400 | Send a position command to the gripper. |
| tool_speed | 401 | Send a velocity command to the gripper. |


#### processCommandSignal
Boolean flag. This flag is on whenever you want to initiate a command. It tells the system object that there is a new command to execute. When a command is currently being executed by the system object, this flag should be set to false.

#### precompute_trj_position
Used alongside the **precomputed_joint_trj** command. It holds an a 2D array of **angular positions** (degrees) for each joint for every frame (1 ms) of the trajectory. It must be the same size as the other precompute parameters.

#### precompute_trj_velocity
Used alongside the **precomputed_joint_trj** command. It holds a 2D array of **angular velocities** (degrees / second) for each joint for every frame (1 ms) of the trajectory. It must be the same size as the other precompute parameters.

#### precompute_trj_acceleration
Used alongside the **precomputed_joint_trj** command. It holds a 2D array of **angular accelerations** (degrees / second ^ squared) for each joint for  every frame (1 ms) of the trajectory. It must be the same size as the other precompute parameters.

#### precompute_trj_timestamp
Used alongside the **precomputed_joint_trj** command. It holds a 2D array of **timestamps** for each joint for every frame (1 ms) of the trajectory. It must be the same size as the other precompute parameters.

#### joint_constraint
Used alongside the **joint_reach** command. It is an array of double values that describes the constraint that will be applied on the joint trajectory.

| Index | Name | Description |
|:-----:|:--------:|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 | Type | The type of constraint that will be applied on the joint trajectory. It is described by `enumJointConstraint.m` located in the `mex-wrapper/enums` folder. <ul><li>no_constraint</li><li>duration</li><li>speed</li></ul> |
| 2 | Duration | The duration (seconds) of the joint trajectory. Used only if the Type of this constraint has been set to **duration**. |
| 3 | Speed | The speed (degrees / second) of the joint trajectory. Used only if the Type of this constraint has been set to **speed**. |


#### joint_cmd
Used alongside the **joint_reach command**. It is an array that contains the position of each joint for a target position. (Index 1 is the joint closest to the robot base and the last index is the joint closest to the gripper)

#### cartesian_constraint
Used alongside the **cartesian_reach** command. It is an array of doubles that describes the constraint that will applied on the Cartesian trajectory.

| Index | Name | Description |
|:-----:|:--------:|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 | Type | The type of constraint that will be applied on the Cartesian trajectory. It is described by `enumCartesianConstraint.m` located in the `mex-wrapper/enums` folder. <ul><li>no_constraint</li><li>duration</li><li>speed</li></ul> |
| 2 | Duration | The duration (seconds) of the joint trajectory. Used only if the Type of this constraint has been set to **duration**. |
| 3 | Speed | The speed of the Cartesian trajectory. Used only if the Type of this constraint has been set to **speed**. It is a list of 2 values, the first one being the translation speed and the second being the orientation speed |

#### cartesian_cmd
Used alongside the **cartesian_reach** command. It is an array of doubles that describes a Cartesian pose. This is the target that the robot will try to reach.

| Index | Name | Description |
|:-----:|:-------:|---------------------------------------------------------|
| 1 | X | Component X of the translation vector in meters. |
| 2 | Y | Component Y of the translation vector in meters. |
| 3 | Z | Component Z of the translation vector in meters. |
| 4 | Theta X | Component Theta X of the orientation vector in degrees. |
| 5 | Theta Y | Component Theta Y of the orientation vector in degrees. |
| 6 | Theta Z | Component Theta Z of the orientation vector in degrees. |

#### tool_constraint
Used alongside the **tool_reach** command. It is the duration constraint, in seconds, that will applied on the tool trajectory. 
> Note: Use the value 0 if you don't want to specify a constraint.

#### tool_cmd
The target command used in the tool trajectory. It is a double value between 0 and 1. 0 being fully opened and 1 being fully closed. 


---

### Outputs


#### status
A Boolean status flag that tells you if the system object is operational (`true`) or in fault (`false`).

#### error_codes
If an error occurred in the current cycle, this is its error code. If this value is 0 than no error occured.

#### cmd_in_progress
The command ID of the current command. It is defined by the the Enum `enumCmd.m` located in the folder `mex-wrapper/enums` (See Input command: **cmdToProcess**)

#### moving_status
The status of the robot.

| Value |           Name          | Description                                                                                                                                                              |
|:-----:|:-----------------------:|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|   0   |   MOVEMENT_STATUS_IDLE  | The robot is idle; it is not moving.                                                                                                                                     |
|   1   | MOVEMENT_STATUS_RUNNING | The robot is currently executing a trajectory; at least one of the joints or the gripper is moving.                                                                       |
|   2   |  MOVEMENT_STATUS_PAUSED | The robot was executing a trajectory but has been paused.                                                                                                                |
|   5   |      PRE_PROCESSING     | The robot is pre-processing a trajectory. This can takes some time depending on the size of the trajectory. The robot will start moving once it has been fully computed. |

#### joint_feedback
The feedback data from each joint of the robot. See [ActuatorsFeedback](data_structure.md#actuatorsfeedback) for details.

#### base_feedback
The feedback data from the robot base. See [BaseFeedback](data_structure.md#basefeedback) for details.

#### tool_feedback
The feedback data from the robot interface module and tool. See [InterconnectFeedback](data_structure.md#interconnectfeedback) for details.

#### vision_info
The configuration data of the vision system. This consists of several fields which themselves have structure:

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| [ColorSettings](data_structure.md#colorsettings)  | color_settings |  Color sensor settings |
| [DepthSettings](data_structure.md#depthsettings)  | depth_settings |  Depth sensor settings |
| [ColorOption](data_structure.md#coloroption)  | color_option |  Color sensor settings |
| [DepthOption](data_structure.md#depthoption)  | depth_option |  Depth sensor settings |
| [Calibration](data_structure#calibration) | calibration_parameters | Intrinsic and extrinsic calibration parameters for color and depth sensors  |


#### is_moving
A Boolean flag that tells if the robot is moving (`true`) or not (`false`).
