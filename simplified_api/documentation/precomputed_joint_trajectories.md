# Precomputed Joint Trajectories

## Overview

When sending a position command to the arm (```joint_reach``` or ```cartesian_reach``` through the System object for example), it always reaches the goal before processing the next one. Hence, the arm velocity between the commands reaches zero.
The Precomputed Joint Trajectories is the way to send the arm through joint-space waypoints that will ensure smooth movement. 

### kObjTrajectoryFeeder object

The kObjTrajectoryFeeder object in the example Simulink model reads a Precomputed Joint Trajectory (computed offline with a motion planner) from a CSV file. and sends it to the kortex System Object. You will most likely create your own object to send your trajectories to the kortex System object. Both the Simulink model and the CSV file used in the exampel can be found in the [latest release of the MATLAB adaptor](https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public%2Fkortex%2Fmatlab%2Fsimplified_API).

### Structure of the input arrays

The input arrays are : 

- precompute_trj_position: 2D array of size [N X] filled with the positions (in degrees) for each joint for each trajectory point, where:
    - N is the number of joints (7 for the Gen3)
    - X is the number of points in the trajectory

- precompute_trj_velocity: 2D array of size [N X] filled with the velocities (in degrees per second) for each joint for each trajectory point, where:
    - N is the number of joints (7 for the Gen3)
    - X is the number of points in the trajectory

- precompute_trj_acceleration: 2D array of size [N X] filled with the accelerations (in degrees per second ^ squared) for each joint for each trajectory point, where:
    - N is the number of joints (7 for the Gen3)
    - X is the number of points in the trajectory

- precompute_trj_timestamp : 2D array of size [1 X] filled with the timestamps (in seconds) for each trajectory point, where:
    - X is the number of points in the trajectory

### Hard limits and conditions to respect

Once sent to the arm, a trajectory is validated and the arm will execute it if it meets all the conditions : 

- **The starting timestamp must be 0.0 seconds.** Timestamps are all relative to the starting timestamp, which must be 0.0.

- **Time increments must be 0.001 seconds between trajectory points.** For now, the arm only accepts trajectories with waypoints every 1ms. 

- **The current position of the arm must match the first trajectory point.** The trajectory planners communicating with the System object have to take into account the current position of the arm for the computation of the trajectories. 

- **The position limit of every joint must be respected for all the trajectory points.** The trajectory will be rejected if any of the points contain an illegal position.

- **The velocity limit of every joint must be respected for all the trajectory points.** The trajectory will be rejected if any of the points contain an illegal velocity. The hard velocity limit for each joint is **50 degree / second.**.

- **The acceleration limit of every joint must be respected for all the trajectory points.** The trajectory will be rejected if any of the points contain an illegal acceleration. The hard acceleration limit for each joint is **1 degree / second ^ squared.**

- **The arrays must contain the same number of joints and number of trajectory points.** The trajectory will be rejected if there is a mismatch in array sizes.

- **Position continuity must be ensured for all trajectory points.** For now, position continuity is the only supported trajectory continuity mode. 

- **The trajectory must not last more than 30 seconds.** For now, it is the maximum supported size for a trajectory (30000 trajectory points).

### Trajectory errors

If the arm encounters errors during the trajectory validation, it will report the errors in a Trajectory Report. The Trajectory Error Report will soon be available in Matlab, and is already available from the Kortex API when calling the GetTrajectoryErrorReport function from the Base service. 