#pragma once

namespace telemetry::types {

inline constexpr const char* kBoolean = "boolean";
inline constexpr const char* kInt64 = "int64";
inline constexpr const char* kString = "string";
inline constexpr const char* kStructSchema = "structschema";

}

namespace telemetry::keys {

inline constexpr const char* kOdometryRobot = "/Odometry/Robot";
inline constexpr const char* kOdometryRaw = "/Odometry/Raw";
inline constexpr const char* kOdometryResetFrom = "/Odometry/ResetFrom";
inline constexpr const char* kOdometryResetTo = "/Odometry/ResetTo";
inline constexpr const char* kOdometryResetSource = "/Odometry/ResetSource";
inline constexpr const char* kOdometryVerticalWheel = "/Odometry/VerticalWheel";
inline constexpr const char* kOdometryHorizontalWheel = "/Odometry/HorizontalWheel";
inline constexpr const char* kOdometryImu = "/Odometry/Imu";

inline constexpr const char* kMotionCommand = "/Motion/Command";
inline constexpr const char* kMotionSource = "/Motion/Source";
inline constexpr const char* kMotionId = "/Motion/Id";
inline constexpr const char* kMotionActive = "/Motion/Active";

inline constexpr const char* kMotorLeftA = "/Motors/LeftA";
inline constexpr const char* kMotorLeftB = "/Motors/LeftB";
inline constexpr const char* kMotorRightA = "/Motors/RightA";
inline constexpr const char* kMotorRightB = "/Motors/RightB";

inline constexpr const char* kMotorLeftAHealth = "/Motors/LeftA/Health";
inline constexpr const char* kMotorLeftBHealth = "/Motors/LeftB/Health";
inline constexpr const char* kMotorRightAHealth = "/Motors/RightA/Health";
inline constexpr const char* kMotorRightBHealth = "/Motors/RightB/Health";

inline constexpr const char* kSystemBattery = "/System/Battery";
inline constexpr const char* kControllerMaster = "/Controller/Master";
inline constexpr const char* kControllerConnected = "/Controller/Connected";
inline constexpr const char* kControllerBattery = "/Controller/Battery";
inline constexpr const char* kDriverStationEnabled = "/DriverStation/Enabled";
inline constexpr const char* kDriverStationAutonomous = "/DriverStation/Autonomous";
inline constexpr const char* kDriverStationConnected = "/DriverStation/Connected";
inline constexpr const char* kDriverStationFieldControl = "/DriverStation/FieldControl";

inline constexpr const char* kAutonSelected = "/Auton/Selected";
inline constexpr const char* kAutonRunning = "/Auton/Running";

inline constexpr const char* kLoggerDropped = "/Logger/Dropped";
inline constexpr const char* kLoggerBytes = "/Logger/Bytes";
inline constexpr const char* kLoggerLoopOverruns = "/Logger/LoopOverruns";

inline constexpr const char* kMetadataBuild = "/Metadata/Build";
inline constexpr const char* kMetadataPorts = "/Metadata/Ports";
inline constexpr const char* kMetadataDrivetrain = "/Metadata/Drivetrain";
inline constexpr const char* kMetadataOdometry = "/Metadata/Odometry";
inline constexpr const char* kMetadataLinearPid = "/Metadata/LinearPID";
inline constexpr const char* kMetadataAngularPid = "/Metadata/AngularPID";
inline constexpr const char* kMetadataDriveCurves = "/Metadata/DriveCurves";
inline constexpr const char* kMetadataUnits = "/Metadata/Units";

}
