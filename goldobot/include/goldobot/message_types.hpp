#pragma once
#include <cstdint>

namespace goldobot {

enum class CommMessageType : uint16_t {
  CommUartStats                           =    0,
  CommUartPing                            =    1,
  Heartbeat                               =    2,
  HeapStats                               =    3,
  Reset                                   =    4,
  GetNucleoFirmwareVersion                =    5,
  TaskStats                               =    6,
  DbgTrace                                =    7,

  MatchTimer                              =   10,
  MatchTimerStart                         =   11,
  MatchEnd                                =   12,
  MatchTimerStop                          =   13,

  DbgGpioGet                              =   20,
  DbgGpioGetStatus                        =   21,
  DbgGpioSet                              =   22,
  DbgPwmSet                               =   23,
  DbgSetStatus                            =   24,

  DbgGoldo                                =   29,

  FpgaReadReg                             =   30,
  FpgaReadRegStatus                       =   31,
  FpgaWriteReg                            =   32,
  SensorsState                            =   33,
  FpgaGpioState                           =   34,
  FpgaReadRegInternal                     =   35,
  FpgaReadAdc                             =   36,
  FpgaReadAdcOut                          =   37,

  ServoAck                                =   40,
  ServoMoveMultiple                       =   41,
  ServoSetEnable                          =   42,
  ServoState                              =   43,
  ServosMoving                            =   44,
  ServoDisableAll                         =   45,
  ServoLiftsCmdRawObsolete                =   46,
  ServoGetState                           =   47,
  ServoLiftDoHomingObsolete               =   48,
  ServoSetMaxTorques                      =   49,

  ODriveRequestPacket                     =   50,
  ODriveResponsePacket                    =   51,
  ODriveTelemetry                         =   52,
  ODriveCommStats                         =   53,

  DynamixelsRequest                       =   60,
  DynamixelsResponse                      =   61,

  PropulsionEnableSet                     =  100,
  PropulsionMotorsEnableSet               =  101,
  PropulsionMotorsVelocitySetpointsSet    =  102,
  PropulsionSetTargetSpeed                =  103,
  PropulsionSetAccelerationLimits         =  104,
  PropulsionSetPose                       =  105,
  PropulsionSetTargetPose                 =  106,
  PropulsionEmergencyStop                 =  107,
  PropulsionClearError                    =  108,
  PropulsionClearCommandQueue             =  109,
  PropulsionSetSimulationMode             =  110,
  PropulsionScopeConfig                   =  111,
  PropulsionMotorsTorqueLimitsSet         =  112,
  PropulsionTransformPose                 =  113,
  PropulsionSetEventSensorsMask           =  114,

  PropulsionTelemetry                     =  120,
  PropulsionTelemetryEx                   =  121,
  PropulsionPose                          =  122,
  PropulsionState                         =  123,
  PropulsionODriveTelemetry               =  124,
  PropulsionScopeData                     =  125,
  PropulsionOdometryStream                =  126,
  PropulsionODriveStream                  =  127,

  PropulsionCommandEvent                  =  130,
  PropulsionControllerEvent               =  131,

  PropulsionExecuteTranslation            =  140,
  PropulsionExecuteMoveTo                 =  141,
  PropulsionExecuteRotation               =  142,
  PropulsionExecutePointTo                =  143,
  PropulsionExecuteFaceDirection          =  144,
  PropulsionExecuteTrajectory             =  145,
  PropulsionMeasureNormal                 =  146,
  PropulsionSetControlLevels              =  147,
  PropulsionEnterManualControl            =  148,
  PropulsionExecuteReposition             =  149,
  PropulsionExitManualControl             =  150,
  PropulsionCalibrateODrive               =  151,
  PropulsionODriveClearErrors             =  152,
  PropulsionExecutePointToBack            =  153,
  PropulsionExecuteUpdateTrajectory       =  154,

  PropulsionODriveStatistics              =  180,
  PropulsionODriveAxisStates              =  181,
  PropulsionODriveAxisErrors              =  182,

  RobotConfigLoadBegin                    =  200,
  RobotConfigLoadChunk                    =  201,
  RobotConfigLoadEnd                      =  202,
  RobotConfigLoadStatus                   =  203,

  OdometryConfigGet                       =  210,
  OdometryConfigGetStatus                 =  211,
  OdometryConfigSet                       =  212,

  PropulsionConfigGet                     =  215,
  PropulsionConfigGetStatus               =  216,
  PropulsionConfigSet                     =  217,

  LiftHomingDone                          =  230,
  LiftSetEnable                           =  231,
  LiftDoHoming                            =  232,
  LiftsCmdRaw                             =  233,

  WatchdogReset                           =  250,
  WatchdogStatus                          =  251,

  UartCommTaskStatistics                  =  300,
  ODriveCommTaskStatistics                =  301,
  PropulsionTaskStatistics                =  302
};

}  // namespace goldobot
