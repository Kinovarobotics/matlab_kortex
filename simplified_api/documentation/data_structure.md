# Data structures


<a name="ActuatorsFeedback"></a>
## ActuatorsFeedback

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t\[MAX_JOINTS\] | status_flags | Status flags |
| uint32_t\[MAX_JOINTS\] | jitter_comm | Jitter from the communication (in microseconds) |
| double\[MAX_JOINTS\] | position | Position of the actuator (in degrees) |
| double\[MAX_JOINTS\] | velocity | Velocity of the actuator (in degrees per second) |
| double\[MAX_JOINTS\] | torque | Torque of the actuator (in Newton * meters) |
| double\[MAX_JOINTS\] | current_motor | Current of the motor (in Amperes) |
| double\[MAX_JOINTS\] | voltage | Voltage of the main board (in Volts) |
| double\[MAX_JOINTS\] | temperature_motor | Motor temperature (maximum of the three (3) phase temperatures in °C) |
| double\[MAX_JOINTS\] | temperature_core | Microcontroller temperature (in degrees Celsius) |
| uint32_t\[MAX_JOINTS\] | fault_bank_a | Fault bank A |
| uint32_t\[MAX_JOINTS\] | fault_bank_b | Fault bank B |
| uint32_t\[MAX_JOINTS\] | warning_bank_a | Warning bank A |
| uint32_t\[MAX_JOINTS\] | warning_bank_b | Warning bank B |


<a name="BaseFeedback"></a>
## BaseFeedback

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t | arm_state | Active state of the arm |
| double | arm_voltage | Arm voltage (in Volts) |
| double | arm_current | Arm current (in Amperes) |
| double | temperature_cpu | CPU temperature (in degree Celcius) |
| double | temperature_ambient | Ambient temperature (in degree Celcius) |
| double\[3\] | imu_acceleration | IMU Measured acceleration (X-Y-Z-Axis) of the base (in meters per second squared) |
| double\[3\] | imu_angular_velocity | IMU Measured angular velocity (X-Y-Z-Axis) of the base (in degrees per second) |
| double\[6\] | tool_pose | Measured Cartesian pose (X-Y-Z-thetaX-thetaY-thetaZ) of the tool (in meters and degrees(**Tait-Bryan, extrinsic ZYX**)) |
| double\[6\] | tool_twist | Measured Cartesian Twist (X-Y-Z-ScrewVector) of the tool (in meters per second) |
| double\[3\] | tool_external_wrench_force | Calculated force in (X-Y-Z) from external wrench (in Newtons) |
| double\[3\] | tool_external_wrench_torque | Calculated torque about (X-Y-Z) from external wrench (in Newton * meters) |
| uint32_t | fault_bank_a | The arm fault flags bank A (0 if no fault) |
| uint32_t | fault_bank_b | The arm fault flags bank B (0 if no fault) |
| uint32_t | warning_bank_a | The arm warning flags bank A (0 if no warning) |
| uint32_t | warning_bank_b | The arm warning flags bank B (0 if no warning) |


<a name="Calibration"></a>
## Calibration

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| [IntrinsicParameters](#intrinsicparameters) | color_intrinsic | Color sensor intrinsic data  |
| [IntrinsicParameters](#intrinsicparameters)  | depth_intrinsic | Depth sensor intrinsic data |
| [ExtrinsicParameters](#extrinsicparameters) | extrinsic | Extrinsic data |


<a name="ColorOption"></a>
## ColorOption

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| double | brightness | Color image brightness: -4.0 to 4.0, step 1.0  |
| double | contrast | Color image contrast: -4.0 to 4.0, step 1.0 |
| double | saturation | Color image saturation setting: -4.0 to 4.0, step 1. |

<a name="ColorSettings"></a>
## ColorSettings

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t | sensor | Sensor  |
| uint32 | resolution | See [Resolution](#resolution) |
| uint32 | frame_rate | See [FrameRate](#framerate)|
| uint32 | bit_rate | See [BitRate](#bitrate) |

<a name="DepthOption"></a>
## DepthOption

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| double | exposure | Controls exposure time of sensor. Setting any value will disable auto exposure: 20.0 to 166000.0, step 20.0  |
| double | gain | Image gain: 16.0 to 248.0, step 1.0 |
| double | visual_preset | Provide access to several recommend sets of option presets for the depth camera: 0.0 to 5.0, step 1.0 |
| double | frames_queue_size | Number of frames the user is allowed to keep per stream. Trying to hold-on to more frames will cause frame-drops: 0.0 to 32.0, step 1.0  |
| double | depth_units | Number of meters represented by a single depth unit: 0.0001 to 0.0100, step 0.000001 |
| double | enable_auto_exposure | Enable / disable color image auto-exposure: 0.0 to 1.0, step 1.0 |
| double | error_polling_enable | Disable error handling: 0.0 to 1.0, step 1.0  |
| double | output_trigger_enable | Enable / disable trigger to be outputed from the camera to any external device on every depth frame: 0.0 to 1.0, step 1.0 |


<a name="DepthSettings"></a>
## DepthSettings

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t | sensor | Sensor  |
| uint32 | resolution | See [Resolution](#resolution) |
| uint32 | frame_rate | See [FrameRate](#framerate)|
| uint32 | bit_rate | See [BitRate](#bitrate) |


<a name="ExtrinsicParameters"></a>
## ExtrinsicParameters

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| [RotationMatrix](#rotationmatrix) | rotation | The rotation matrix from depth to color camera |
| [TranslationVector](#translationvector) | translation | The translation vector from depth to color camera |

## ErrorStructMatrix
This is a data structure returned by most of the functions of the MEX interface. It consists of an error code and an error sub-code.

|   Name   |   Type   | Description                                                                                                                                                                 |
|:--------:|:--------:|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| uint32_t |   code   | A general [error code](#error-code-list) associated with this error. Most users will not need to work with this value because it does not tell much and is used internally in the Kortex API. |
| uint32_t | sub_code | An [error sub code](#sub-error-code-list) that gives more information about the error. Most users will prefer to use this value instead of the general code.                        

<a name="GripperFeedback"></a>
## GripperFeedback

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t | feedback_id | MessageId |
| uint32_t | status_flags | Status flags (see `GripperConfig.RobotiqGripperStatusFlags`) |
| uint32_t | fault_bank_a | Fault bank A (see `GripperConfig.SafetyIdentifier`) |
| uint32_t | fault_bank_b | Fault bank B (see `GripperConfig.SafetyIdentifier`) |
| uint32_t | warning_bank_a | Warning bank A (see `GripperConfig.SafetyIdentifier`) |
| uint32_t | warning_bank_b | Warning bank B (see `GripperConfig.SafetyIdentifier`) |
| uint32_t | motor_count | Motor count on the gripper. |
| uint32_t\[GRIPPER_MAX_MOTOR_COUNT\] | motor_id | Motor ID |
| double\[GRIPPER_MAX_MOTOR_COUNT\] | motor_position | Position of the gripper fingers in percentage (0-100%) |
| double\[GRIPPER_MAX_MOTOR_COUNT\] | motor_velocity | Velocity of the gripper fingers in percentage (0-100%) |
| double\[GRIPPER_MAX_MOTOR_COUNT\] | motor_current_motor | Current comsumed by the gripper motor (mA) |
| double\[GRIPPER_MAX_MOTOR_COUNT\] | motor_voltage | Motor Voltage (V) |
| double\[GRIPPER_MAX_MOTOR_COUNT\] | motor_temperature_motor | Motor temperature. (degrees Celsius) |


<a name="InterconnectFeedback"></a>
## InterconnectFeedback

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t | feedback_id | MessageId |
| uint32_t | status_flags | Status flags |
| uint32_t | jitter_comm | Jitter from the communication (in microsecond) |
| double | imu_acceleration_x | IMU Measured acceleration (X-Axis) of the interconnect (in meters per second ^ squared) |
| double | imu_acceleration_y | IMU Measured acceleration (Y-Axis) of the interconnect (in meters per second ^ squared) |
| double | imu_acceleration_z | IMU Measured acceleration (Z-Axis) of the interconnect (in meters per second ^ squared) |
| double | imu_angular_velocity_x | IMU Measured angular velocity (X-Axis) of the interconnect (in degrees per second) |
| double | imu_angular_velocity_y | IMU Measured angular velocity (Y-Axis) of the interconnect (in degrees per second) |
| double | imu_angular_velocity_z | IMU Measured angular velocity (Z-Axis) of the interconnect (in degrees per second) |
| double | voltage | Voltage of the main board (in Volt) |
| double | temperature_core | Microcontroller temperature (in degrees Celsius) |
| uint32_t | fault_bank_a | Fault bank A (see `InterconnectConfig.SafetyIdentifier`) |
| uint32_t | fault_bank_b | Fault bank B (see `InterconnectConfig.SafetyIdentifier`) |
| uint32_t | warning_bank_a | Warning bank A (see `InterconnectConfig.SafetyIdentifier`) |
| uint32_t | warning_bank_b | Warning bank B (see `InterconnectConfig.SafetyIdentifier`) |
| double | difference_count_a |  |
| double | difference_count_b |  |
| [GripperFeedback](#gripperfeedback) | gripper_feedback | Feedback data from gripper (if attached) |


<a name="IntrinsicParameters"></a>
## IntrinsicParameters

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| uint32_t | sensor | The sensor on which to perform the focus action |
| uint32_t | resolution | The resolution for which the parameters apply to |
| double | principal_point_x | Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge |
| double | principal_point_y | Vertical coordinate of the principal point of the image, as a pixel offset from the top edge |
| double | focal_length_x | Focal length of the image plane, as a multiple of pixel width |
| double | focal_length_y | Focal length of the image plane, as a multiple of pixel height |
| double | k1 | First radial distortion coefficient |
| double | k2 | Second radial distortion coefficient |
| double | k3 | Third radial distortion coefficient |
| double | p1 | First tangential distortion coefficient |
| double | p2 | Second tangential distortion coefficient |


<a name="RotationMatrix"></a>
## RotationMatrix

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| RotationMatrixRow | row1 | First rotation matrix row |
| RotationMatrixRow | row2 | Second rotation matrix row |
| RotationMatrixRow | row3 | Third rotation matrix row |

<a name="RotationMatrixRow"></a>
## RotationMatrixRow

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| float | column1 | Value between -1.0 and 1.0 |
| float | column2 | Value between -1.0 and 1.0 |
| float | column3 | Value between -1.0 and 1.0 |

<a name="TranslationVector"></a>
## TranslationVector

| Type | Name | Description |
|:-----:|:----:|:-----------:|
| float | x | Translation in meters in the x axis |
| float | y | Translation in meters in the y axis |
| float | z | Translation in meter



# Enumerations


<a name="AdmittanceMode"></a>
## AdmittanceMode

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | UNSPECIFIED_ADMITTANCE_MODE | Unspecified admittance mode |
|   1   | CARTESIAN | Cartesian admittance mode |
|   2   | JOINT | Joint admittance mode |
|   3   | NULL_SPACE | Null space admittance mode |
|   4   | DISABLED | No admittance |


<a name="bitRate"></a>
## BitRate

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | BITRATE_UNSPECIFIED | Unspecified bit rate (supported on depth sensor only) |
|   1   | BITRATE_10_MBPS | 10 Mbps maximum bit rate (supported on color sensor only) |
|   2   | BITRATE_15_MBPS | 15 Mbps maximum bit rate (supported on color sensor only) |
|   3   | BITRATE_20_MBPS | 20 Mbps maximum bit rate (supported on color sensor only) |
|   4   | BITRATE_25_MBPS | 25 Mbps maximum bit rate (supported on color sensor only) |


<a name="CartesianConstraintType"></a>
## CartesianConstraintType

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | CARTESIAN_NO_CONSTRAINT |  |
|   1   | CARTESIAN_CONSTRAINT_DURATION | (we only support CARTESIAN_CONSTRAINT_SPEED for now) |
|   2   | CARTESIAN_CONSTRAINT_SPEED |  |


## Error code list

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | ERROR_NONE |  No error |
|   1   | ERROR_PROTOCOL_SERVER |  Protocol server error |
|   2   | ERROR_PROTOCOL_CLIENT |  Protocol client error |
|   3   | ERROR_DEVICE |  Device error |
|   4   | ERROR_INTERNAL |  Internal error |




<a name="FrameRate"></a>
## FrameRate

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | FRAMERATE_UNSPECIFIED | Unspecified frame rate |
|   1   | FRAMERATE_6_FPS | 6 frames per second (supported on depth sensor only) |
|   2   | FRAMERATE_15_FPS | 15 frames per second |
|   3   | FRAMERATE_30_FPS | 30 frame per second |

<a name="JointConstraintType"></a>
## JointConstraintType

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | UNSPECIFIED_JOINT_CONSTRAINT | Unspecified joint constraint |
|   1   | JOINT_CONSTRAINT_DURATION | Duration constraint (in seconds) |
|   2   | JOINT_CONSTRAINT_SPEED | Speed constraint (in meters per second) |


<a name="MovementStatus"></a>
## MovementStatus

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | MOVEMENT_STATUS_IDLE |  |
|   1   | MOVEMENT_STATUS_RUNNING |  |
|   2   | MOVEMENT_STATUS_PAUSED |  |
|   3   | MOVEMENT_STATUS_ABORTED |  |
|   4   | MOVEMENT_STATUS_COMPLETED |  |
|   5   | PRE_PROCESSING |  |


<a name="Option"></a>
## Option

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | OPTION_UNSPECIFIED | Unspecified Option |
|   1   | OPTION_BACKLIGHT_COMPENSATION | Enable / disable color backlight compensation (unsupported) |
|   2   | OPTION_BRIGHTNESS | Color image brightness (supported on color sensor only: -4.0 to 4.0, step 1.0) |
|   3   | OPTION_CONTRAST | Color image contrast (supported on color sensor only: -4.0 to 4.0, step 1.0) |
|   4   | OPTION_EXPOSURE | Controls exposure time of color camera. Setting any value will disable auto exposure (supported on depth sensor only: 20.0 to 166000.0, step 20.0) |
|   5   | OPTION_GAIN | Color image gain (supported on depth sensor only: 16.0 to 248.0, step 1.0) |
|   6   | OPTION_GAMMA | Color image gamma setting (unsupported) |
|   7   | OPTION_HUE | Color image hue (unsupported) |
|   8   | OPTION_SATURATION | Color image saturation setting (supported on color sensor only: -4.0 to 4.0, step 1.0) |
|   9   | OPTION_SHARPNESS | Color image sharpness setting (unsupported) |
|   10   | OPTION_WHITE_BALANCE | Controls white balance of color image. Setting any value will disable auto white balance (unsupported) |
|   11   | OPTION_ENABLE_AUTO_EXPOSURE | Enable / disable color image auto-exposure (supported on depth sensor only: 0.0 to 1.0, step 1.0) |
|   12   | OPTION_ENABLE_AUTO_WHITE_BALANCE | Enable / disable color image auto-white-balance (unsupported) |
|   13   | OPTION_VISUAL_PRESET | Provide access to several recommend sets of option presets for the depth camera (supported on depth sensor only: 0.0 to 5.0, step 1.0) |
|   14   | OPTION_LASER_POWER | Power of the projector, with 0 meaning projector off (unsupported) |
|   15   | OPTION_ACCURACY | Sets the number of patterns projected per frame. The higher the accuracy value the more patterns projected (unsupported) |
|   16   | OPTION_MOTION_RANGE | Motion vs. Range trade-off, with lower values allowing for better motion sensitivity and higher values allowing for better depth range (unsupported) |
|   17   | OPTION_FILTER_OPTION | Sets the filter to apply to each depth frame. Each one of the filter is optimized per the application requirements (unsupported) |
|   18   | OPTION_CONFIDENCE_THRESHOLD | The confidence level threshold used by the Depth algorithm pipe to set whether a pixel will get a valid range or will be marked with invalid range (unsupported) |
|   19   | OPTION_EMITTER_ENABLED | Laser Emitter enabled (unsupported) |
|   20   | OPTION_FRAMES_QUEUE_SIZE | Number of frames the user is allowed to keep per stream. Trying to hold-on to more frames will cause frame-drops (supported on depth sensor only: 0.0 to 32.0, step 1.0) |
|   21   | OPTION_TOTAL_FRAME_DROPS | Total number of detected frame drops from all streams (unsupported) |
|   22   | OPTION_AUTO_EXPOSURE_MODE | Auto-Exposure modes: Static, Anti-Flicker and Hybrid (unsupported) |
|   23   | OPTION_POWER_LINE_FREQUENCY | Power Line Frequency control for anti-flickering Off/50Hz/60Hz/Auto (unsupported) |
|   24   | OPTION_ASIC_TEMPERATURE | Current Asic Temperature (supported on depth sensor only: Read Only -40.0 to 125.0) |
|   25   | OPTION_ERROR_POLLING_ENABLED | Disable error handling (supported on depth sensor only: 0.0 to 1.0, step 1.0) |
|   26   | OPTION_PROJECTOR_TEMPERATURE | Current Projector Temperature (unsupported) |
|   27   | OPTION_OUTPUT_TRIGGER_ENABLED | Enable / disable trigger to be outputed from the camera to any external device on every depth frame (supported on depth sensor only: 0.0 to 1.0, step 1.0) |
|   28   | OPTION_MOTION_MODULE_TEMPERATURE | Current Motion-Module Temperature (unsupported) |
|   29   | OPTION_DEPTH_UNITS | Number of meters represented by a single depth unit (supported on depth sensor only: 0.0001 to 0.0100, step 0.000001) |
|   30   | OPTION_ENABLE_MOTION_CORRECTION | Enable/Disable automatic correction of the motion data (unsupported) |
|   31   | OPTION_AUTO_EXPOSURE_PRIORITY | Allows sensor to dynamically ajust the frame rate depending on lighting conditions (unsupported) |
|   32   | OPTION_COLOR_SCHEME | Color scheme for data visualization (unsupported) |
|   33   | OPTION_HISTOGRAM_EQUALIZATION_ENABLED | Perform histogram equalization post-processing on the depth data (unsupported) |
|   34   | OPTION_MIN_DISTANCE | Minimal distance to the target (unsupported) |
|   35   | OPTION_MAX_DISTANCE | Maximum distance to the target (unsupported) |
|   36   | OPTION_TEXTURE_SOURCE | Texture mapping stream unique ID (unsupported) |
|   37   | OPTION_FILTER_MAGNITUDE | The 2D-filter effect. The specific interpretation is given within the context of the filter (unsupported) |
|   38   | OPTION_FILTER_SMOOTH_ALPHA | 2D-filter parameter controls the weight/radius for smoothing (unsupported) |
|   39   | OPTION_FILTER_SMOOTH_DELTA | 2D-filter range/validity threshold (unsupported) |
|   40   | OPTION_HOLES_FILL | Enhance depth data post-processing with holes filling where appropriate (unsupported) |
|   41   | OPTION_STEREO_BASELINE | The distance in mm between the first and the second imagers in stereo-based depth cameras (supported on depth sensor only: 55.241055 to 55.241055, step 0.0) |
|   42   | OPTION_AUTO_EXPOSURE_CONVERGE_STEP | Allows dynamically ajust the converge step value of the target exposure in Auto-Exposure algorithm (unsupported) |

<a name="Resolution"></a>
## Resolution

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | RESOLUTION_UNSPECIFIED | Unspecified resolution |
|   1   | RESOLUTION_320x240 | 320 x 240 pixels (supported on color sensor only) |
|   2   | RESOLUTION_424x240 | 424 x 240 pixels (supported on depth sensor only) |
|   3   | RESOLUTION_480x270 | 480 x 270 pixels (supported on depth sensor only) |
|   4   | RESOLUTION_640x480 | 640 x 480 pixels (supported on color sensor only) |
|   5   | RESOLUTION_1280x720 | 1280 x 720 pixels (HD) (supported on color sensor only) |
|   6   | RESOLUTION_1920x1080 | 1920 x 1080 pixels (full HD) (supported on color sensor only) |


<a name="Sensor"></a>
## Sensor

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | SENSOR_UNSPECIFIED | Unspecified Sensor |
|   1   | SENSOR_COLOR | Select the Vision module color sensor |
|   2   | SENSOR_DEPTH | Select the Vision module depth sensor |

<a name="ServoingMode"></a>
## ServoingMode

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | UNSPECIFIED_SERVOING_MODE | Unspecified servoing mode |
|   2   | SINGLE_LEVEL_SERVOING | Single-level servoing |
|   3   | LOW_LEVEL_SERVOING | Low-level servoing |
|   4   | BYPASS_SERVOING | Bypass mode |

## Sub error code list

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | SUB_ERROR_NONE |  No sub error |
|   1   | METHOD_FAILED |  Method returned a failure status (generic error) |
|   2   | UNIMPLEMENTED |  Unimplemented method |
|   3   | INVALID_PARAM |  Invalid parameter |
|   4   | UNSUPPORTED_SERVICE |  Service not recognized |
|   5   | UNSUPPORTED_METHOD |  Method not recognized |
|   6   | TOO_LARGE_ENCODED_FRAME_BUFFER |  Encoded frame bigger than what transport permits |
|   7   | FRAME_ENCODING_ERR |  Unable to encode frame |
|   8   | FRAME_DECODING_ERR |  Unable to decode frame |
|   9   | INCOMPATIBLE_HEADER_VERSION |  Frame header version differs from what is expected and is considered incompatible |
|   10   | UNSUPPORTED_FRAME_TYPE |  Unrecognized frame type |
|   11   | UNREGISTERED_NOTIFICATION_RECEIVED |  Server receiving unregistered notification |
|   12   | INVALID_SESSION |  Session not recognized |
|   13   | PAYLOAD_DECODING_ERR |  Unable to decode payload |
|   14   | UNREGISTERED_FRAME_RECEIVED |  Client received a response for which it did not send an RPC call |
|   15   | INVALID_PASSWORD |  Password does not match specified user |
|   16   | USER_NOT_FOUND |  Unrecognized user |
|   17   | ENTITY_NOT_FOUND |  Cannot find entity |
|   18   | ROBOT_MOVEMENT_IN_PROGRESS |  Robot refuses new control command because robot movement in progress |
|   19   | ROBOT_NOT_MOVING |  Robot refuses stop command because robot is not moving |
|   20   | NO_MORE_STORAGE_SPACE |  Unable to execute because no more storage |
|   21   | ROBOT_NOT_READY |  Robot initialization is not complete |
|   22   | ROBOT_IN_FAULT |  Robot in fault |
|   23   | ROBOT_IN_MAINTENANCE |  Robot in maintenance |
|   24   | ROBOT_IN_UPDATE_MODE |  Robot in update |
|   25   | ROBOT_IN_EMERGENCY_STOP |  Robot in emergency stop state |
|   26   | SINGLE_LEVEL_SERVOING |  Robot is in single-level servoing mode |
|   27   | LOW_LEVEL_SERVOING |  Robot is in low-level servoing mode |
|   28   | MAPPING_GROUP_NON_ROOT |  Trying to add a non-root MapGroup to Mapping |
|   29   | MAPPING_INVALID_GROUP |  Trying to add an invalid or non-existent MapGroup to Mapping |
|   30   | MAPPING_INVALID_MAP |  Trying to add an invalid or non-existent Map to Mapping |
|   31   | MAP_GROUP_INVALID_MAP |  Trying to add an invalid or non-existent Map to MapGroup |
|   32   | MAP_GROUP_INVALID_PARENT |  Trying to add a MapGroup under an invalid parent |
|   33   | MAP_GROUP_INVALID_CHILD |  Trying to add an invalid or non-existent to MapGroup |
|   34   | MAP_GROUP_INVALID_MOVE |  Trying to change a MapGroup's parent: move not supported |
|   35   | MAP_IN_USE |  Deleting a Map used in a Mapping or MapGroup |
|   36   | WIFI_CONNECT_ERROR |  Unable to connect to specified Wifi network |
|   37   | UNSUPPORTED_NETWORK_TYPE |  Unsupported network type |
|   38   | TOO_LARGE_ENCODED_PAYLOAD_BUFFER |  Encoded payload bigger than what transport permits |
|   39   | UPDATE_PERMISSION_DENIED |  Attempting update command on non-updatable entity |
|   40   | DELETE_PERMISSION_DENIED |  Attempting delete command on non-deletable entity |
|   41   | DATABASE_ERROR |  Internal DB error |
|   42   | UNSUPPORTED_OPTION |  Option not supported |
|   43   | UNSUPPORTED_RESOLUTION |  Resolution not supported |
|   44   | UNSUPPORTED_FRAME_RATE |  Frame rate not supported |
|   45   | UNSUPPORTED_BIT_RATE |  Bit rate not supported |
|   46   | UNSUPPORTED_ACTION |  Action not supported (generic, when an action is not supported for a particular item) |
|   47   | UNSUPPORTED_FOCUS_ACTION |  Focus action not supported |
|   48   | VALUE_IS_ABOVE_MAXIMUM |  Specified value is above the supported maximum |
|   49   | VALUE_IS_BELOW_MINIMUM |  Specified value is below the supported minimum |
|   50   | DEVICE_DISCONNECTED |  Device is not connected |
|   51   | DEVICE_NOT_READY |  Device is not ready |
|   52   | INVALID_DEVICE |  Device id is invalid during bridging |
|   53   | SAFETY_THRESHOLD_REACHED |  Safety threshold is reached therefore safety is on |
|   54   | INVALID_USER_SESSION_ACCESS |  Service or function access not allowed: out of session or level access |
|   55   | CONTROL_MANUAL_STOP |  Manually stopped sequence or action |
|   56   | CONTROL_OUTSIDE_WORKSPACE |  Commanded Cartesian position is outside of robot workspace |
|   57   | CONTROL_ACTUATOR_COUNT_MISMATCH |  Number of constraint sent does not correspond to number of actuator (ex: joint speed) |
|   58   | CONTROL_INVALID_DURATION |  Duration constraint is too short. The robot would need out of limit speeds/accelerations to reach this duration. |
|   59   | CONTROL_INVALID_SPEED |  Speed constraint is negative |
|   60   | CONTROL_LARGE_SPEED |  Speed constraint is too high (exceed speed limit of leads to high acceleration) |
|   61   | CONTROL_INVALID_ACCELERATION |  Speed constraint is too high or duration constraint too short and leads to high acceleration |
|   62   | CONTROL_INVALID_TIME_STEP |  Refresh rate is smaller than the duration of the trajectory |
|   63   | CONTROL_LARGE_SIZE |  Duration of the trajectory is more than 100s. The length of the trajectory is limited to 100000 points to avoid saturating the base memory. |
|   64   | CONTROL_WRONG_MODE |  Control mode is not a trajectory mode |
|   65   | CONTROL_JOINT_POSITION_LIMIT |  Commanded configuration contains at least one actuator which is out of its physical limits |
|   66   | CONTROL_NO_FILE_IN_MEMORY |  Trajectory is not computed and try to be started |
|   67   | CONTROL_INDEX_OUT_OF_TRAJECTORY |  Attempting to read a point of the trajectory with an index higher than the number of point in trajectory point list. |
|   68   | CONTROL_ALREADY_RUNNING |  Trajectory is already running |
|   69   | CONTROL_WRONG_STARTING_POINT |  Robot is not on the first point of the trajectory when we try to start the trajectory. This can happen if there is a motion between the moment when trajectory is computed and when it is started. |
|   70   | CONTROL_CARTESIAN_CANNOT_START |  Cannot start |
|   71   | CONTROL_UNDEFINED_CONSTRAINT |  Kontrol library is not initialized |
|   72   | CONTROL_UNINITIALIZED |  Contraint sent is not defined |
|   73   | CONTROL_NO_ACTION |  Action does not exist |
|   74   | CONTROL_UNDEFINED |  Undefined error |
|   75   | WRONG_SERVOING_MODE |  Robot is in not in the right servoing mode |
|   76   | CONTROL_WRONG_STARTING_SPEED |  Robot is not at the right speed when starting a new trajectory. |
|   100   | USERNAME_LENGTH_EXCEEDED |  User profile username length exceeds maximum allowed length |
|   101   | FIRSTNAME_LENGTH_EXCEEDED |  User profile first name length exceeds maximum allowed length |
|   102   | LASTNAME_LENGTH_EXCEEDED |  User profile last name length exceeds maximum allowed length |
|   103   | PASSWORD_LENGTH_EXCEEDED |  User profile password length exceeds maximum allowed length |
|   104   | USERNAME_ALREADY_EXISTS |  User profile username already in use by another profile |
|   105   | USERNAME_EMPTY |  User profile empty username not allowed |
|   106   | PASSWORD_NOT_CHANGED |  Change password both passwords are the same |
|   107   | MAXIMUM_USER_PROFILES_USED |  Maximum number of user profiles in use |
|   108   | ROUTER_UNVAILABLE |  The client router is currently unavailable. This can happen if an API method is called after the router has been deactivated via the method SetActivationStatus. |
|   120   | ADDRESS_NOT_IN_VALID_RANGE |  IP Address not valid against netmask |
|   130   | SESSION_NOT_IN_CONTROL |  Trying to perform command from a non-controlling session in single-level mode |
|   131   | METHOD_TIMEOUT |  Timeout occured during method execution |
|   132   | UNSUPPORTED_ROBOT_CONFIGURATION |  Product Configuration setter method failed because changing this parameter is unsupported on your robot model |
|   133   | NVRAM_READ_FAIL |  Failed to read in NVRAM. |
|   134   | NVRAM_WRITE_FAIL |  Failed to write in NVRAM. |

<a name="ToolMode"></a>
## ToolMode

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | UNSPECIFIED_GRIPPER_MODE | Unspecified gripper mode |
|   1   | GRIPPER_FORCE | Force control (in Newton) (not implemented yet) |
|   2   | GRIPPER_SPEED | Speed control (in meters per second) |
|   3   | GRIPPER_POSITION | Position control (in meters) |

<a name="TrajectoryContinuityMode"></a>
## TrajectoryContinuityMode

| Value | Name |Description |
|:-----:|:----:|:-----------:|
|   0   | TRAJECTORY_CONTINUITY_MODE_UNDEFINED | undefined |
|   1   | TRAJECTORY_CONTINUITY_MODE_POSITION | Position continuity only |
|   2   | TRAJECTORY_CONTINUITY_MODE_SPEED | Position and speed continuity |
|   3   | TRAJECTORY_CONTINUITY_MODE_ACCELERATION | Position, speed and acceleration continuity |