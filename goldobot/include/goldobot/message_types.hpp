#pragma once
#include <cstdint>

namespace goldobot {

enum class CommMessageType : uint16_t {
  CommUartStats,
  CommUartPing,
  Heartbeat,
  HeapStats,
  Reset,
  GetNucleoFirmwareVersion,
  TaskStats,
  DbgTrace,

  MatchTimer = 10,
  MatchTimerStart,
  MatchEnd,
  MatchTimerStop,

  DbgGpioGet = 20,
  DbgGpioGetStatus,
  DbgGpioSet,
  DbgPwmSet,
  DbgSetStatus,

  DbgGoldo = 29,

  FpgaReadReg = 30,
  FpgaReadRegStatus,
  FpgaWriteReg,
  SensorsState,
  FpgaGpioState,
  FpgaReadRegInternal,

  ServoAck = 40,
  ServoMoveMultiple,
  ServoSetEnable,
  ServoState,
  ServosMoving,
  ServoDisableAll,
  ServoSetLiftEnable,
  ServoGetState,
  ServoLiftDoHoming,
  ServoSetMaxTorques,

  ODriveRequestPacket = 50,
  ODriveResponsePacket,
  ODriveTelemetry,
  ODriveCommStats,

  DynamixelsRequest = 60,
  DynamixelsResponse,

  PropulsionEnableSet = 100,
  PropulsionMotorsEnableSet,
  PropulsionMotorsVelocitySetpointsSet,
  PropulsionSetTargetSpeed,
  PropulsionSetAccelerationLimits,
  PropulsionSetPose,
  PropulsionSetTargetPose,
  PropulsionEmergencyStop,
  PropulsionClearError,
  PropulsionClearCommandQueue,
  PropulsionSetSimulationMode,
  PropulsionScopeConfig,
  PropulsionMotorsTorqueLimitsSet,
  PropulsionTransformPose,
  PropulsionSetEventSensorsMask,

  PropulsionTelemetry = 120,
  PropulsionTelemetryEx,
  PropulsionPose,
  PropulsionState,
  PropulsionODriveTelemetry,
  PropulsionScopeData,
  PropulsionOdometryStream,
  PropulsionODriveStream,
  PropulsionCommandEvent = 130,
  PropulsionControllerEvent,

  PropulsionExecuteTranslation = 140,
  PropulsionExecuteMoveTo,
  PropulsionExecuteRotation,
  PropulsionExecutePointTo,
  PropulsionExecuteFaceDirection,
  PropulsionExecuteTrajectory,
  PropulsionMeasureNormal,
  PropulsionSetControlLevels,
  PropulsionEnterManualControl,
  PropulsionExecuteReposition,
  PropulsionExitManualControl,
  PropulsionCalibrateODrive,
  PropulsionODriveClearErrors,
  PropulsionExecutePointToBack,
  PropulsionExecuteUpdateTrajectory,

  PropulsionODriveStatistics = 180,
  PropulsionODriveAxisStates,
  PropulsionODriveAxisErrors,

  RobotConfigLoadBegin = 200,
  RobotConfigLoadChunk,
  RobotConfigLoadEnd,
  RobotConfigLoadStatus,

  OdometryConfigGet = 210,
  OdometryConfigGetStatus,
  OdometryConfigSet,

  PropulsionConfigGet = 215,
  PropulsionConfigGetStatus,
  PropulsionConfigSet,

  WatchdogReset = 250,
  WatchdogStatus,

  UartCommTaskStatistics = 300,
  ODriveCommTaskStatistics = 301,
  PropulsionTaskStatistics = 302
};

// task ids for watchdog
// 0: main, 1: propulsion, 2: fpga, 3, odrive_comm, 4: dynamixels_comm, 5: servos

/*
enum class CommMessageType2 : uint16_t {
  // General messages
  Sync = 0,       // "goldobot" synchronization message, used to synchronize stream parser
  Heartbeat = 1,  // Current OS time in ms as uint32, sent every 10th of second
  Reset = 2,      // Sent once on startup
  CommStats = 3,
  DbgPrintf = 4,
  GetNucleoFirmwareVersion = 5,
  Ping = 6,
  HeapStats = 7,
  // Propulsion telemetry
  PropulsionTelemetry = 8,  //
  PropulsionTelemetryEx = 9,
  PropulsionStateChange = 10,
  PropulsionPose = 11,
  // Match events
  MatchStateChange = 15,
  SensorsChange = 20,
  GPIODebug = 21,
  SequenceEvent = 22,
  MatchRemainingTime = 23,

  NucleoLog = 30,

  DebugGoldo = 31,

  // Commands
  CmdEmergencyStop = 32,  // Order an emergency stop
  CmdSelectSide = 33,     // Select side. payload is an unsigned byte, 0=green, 1=orange
  CmdEnterDebugMode = 34,
  CmdExitDebugMode = 35,
  MainSequenceBeginLoad = 40,
  MainSequenceEndLoad = 41,
  MainSequenceLoadData = 42,
  MainSequenceStartSequence = 43,
  SetMatchState = 44,
  MainSequenceAbortSequence = 45,
  MainSequenceLoadStatus = 46,
  // Debug mode messages
  // Robot configuration
  DbgGetOdometryConfig = 64,
  DbgSetOdometryConfig = 65,
  DbgGetPropulsionConfig = 66,
  DbgSetPropulsionConfig = 67,

  // Dynamixels
  DynamixelsPing = 70,
  DynamixelsRead = 71,
  DynamixelsWrite = 72,
  DynamixelsRegWrite = 73,
  DynamixelsAction = 74,
  DynamixelsReboot = 75,
  DynamixelsSyncWrite = 76,
  DynamixelsReadStatus = 77,

  // Propulsion debug
  DbgSetMotorsEnable = 80,
  DbgSetMotorsPwm = 81,
  DbgSetPropulsionEnable = 82,
  DbgPropulsionSetPose = 83,
  DbgPropulsionTest = 84,
  DbgPropulsionExecuteTrajectory = 85,
  DbgPropulsionExecuteRotation = 86,
  DbgPropulsionExecuteReposition = 87,
  DbgPropulsionExecutePointTo = 88,
  DbgPropulsionExecuteMoveTo = 89,
  PropulsionStateChanged = 90,
  PropulsionExecuteTranslation = 91,
  PropulsionEnterManualControl = 92,
  PropulsionExitManualControl = 93,
  PropulsionSetTargetPose = 94,
  PropulsionSetControlLevels = 95,
  PropulsionExecuteFaceDirection = 96,
  PropulsionMeasureNormal = 97,
  PropulsionClearCommandQueue = 98,
  PropulsionClearError = 99,
  PropulsionSetAdversaryDetectionEnable = 100,
  PropulsionMeasurePoint = 101,

  DbgReset = 127,

  DbgArmsSetPose = 160,
  DbgArmsSetCommand = 161,
  DbgArmsSetTorques = 162,
  DbgArmsSetSequences = 163,
  DbgArmsExecuteSequence = 164,
  DbgArmsGoToPosition = 165,
  ArmsStateChange = 166,
  ArmsShutdown = 167,

  DbgRobotSetCommand = 176,
  DbgRobotSetPoint = 177,
  DbgRobotSetSequence = 178,
  DbgRobotExecuteSequence = 179,
  DbgRobotSetTrajectoryPoint = 180,

  // FPGA
  FpgaGetVersion = 256,
  FpgaDbgReadReg = 257,
  FpgaDbgWriteReg = 258,
  FpgaDbgReadRegCrc = 259,
  FpgaDbgGetErrCnt = 260,
  FpgaCmdServo = 272,
  FpgaCmdDCMotor = 288,
  FpgaCmdPumpR = 289,
  FpgaCmdPumpL = 290,
  FpgaCmdConveyorBelt = 291,
  FpgaCmdStepper = 304,
  FpgaGetStepperPos = 305,
  FpgaColumnsCalib = 320,
  FpgaColumnsMove = 321,
  FpgaColumnsSetOffset = 322,
  FpgaServoState = 323,

  RobotBeginLoadConfig = 400,
  RobotLoadConfig = 401,
  RobotEndLoadConfig = 402,
  RobotEndLoadConfigStatus = 403,

  // ODrive
  ODriveRequestPacket = 410,
  ODriveResponsePacket = 411,

  // HAL
  HalGpioGet = 500,
  HalGpioSet,
  HalPwmSet,

  RplidarStart = 1024,
  RplidarStop = 1025,

  RplidarRobotDetection = 1280,

};*/
}  // namespace goldobot
