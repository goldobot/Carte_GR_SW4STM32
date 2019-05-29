#pragma once
#include <cstdint>


namespace goldobot
{
	enum class CommMessageType : uint16_t
	{
		// General messages
		Sync=0, // "goldobot" synchronization message, used to synchronize stream parser
		Heartbeat=1, // Current OS time in ms as uint32, sent every second
		Reset=2, // Sent once on startup
		CommStats=3,
		DbgPrintf=4,
		// Propulsion telemetry
		PropulsionTelemetry=8, //
		PropulsionTelemetryEx=9,
		PropulsionStateChange=10,
		PropulsionPose=11,
		// Match events
		MatchStateChange=15,
		SensorsChange=20,
		GPIODebug=21,
		// Commands
		CmdEmergencyStop=32, // Order an emergency stop
		CmdSelectSide=33, // Select side. payload is an unsigned byte, 0=green, 1=orange
		CmdEnterDebugMode=34,
		CmdExitDebugMode=35,
		MainSequenceBeginLoad=40,
		MainSequenceEndLoad=41,
		MainSequenceLoadData=42,
		MainSequenceStartSequence=43,
		SetMatchState=44,
		// Debug mode messages
		// Robot configuration
		DbgGetOdometryConfig=64,
		DbgSetOdometryConfig=65,
		DbgGetPropulsionConfig=66,
		DbgSetPropulsionConfig=67,

		// Dynamixels debug
		DbgDynamixelsList=72,
		DbgDynamixelDescr=73,
		DbgDynamixelSetTorqueEnable=74,
		DbgDynamixelSetGoalPosition=75,
		DbgDynamixelSetTorqueLimit=76,
		DbgDynamixelGetRegisters=77,
		DbgDynamixelSetRegisters=78,

		// Propulsion debug
		DbgSetMotorsEnable=80,
		DbgSetMotorsPwm=81,
		DbgSetPropulsionEnable=82,
		DbgPropulsionSetPose=83,
		DbgPropulsionTest=84,
		DbgPropulsionExecuteTrajectory=85,
		DbgPropulsionExecuteRotation=86,
		DbgPropulsionExecuteReposition=87,
		DbgPropulsionExecutePointTo=88,
		DbgPropulsionExecuteMoveTo=89,
		PropulsionStateChanged=90,
		PropulsionExecuteTranslation=91,
		PropulsionEnterManualControl=92,
		PropulsionExitManualControl=93,
		PropulsionSetTargetPose=94,
		PropulsionSetControlLevels=95,

		PropulsionClearError=99,


		DbgMiscRepositionStartGreen=100,
		DbgReset=127,

		DbgArmsSetPose=160,
		DbgArmsSetCommand=161,
		DbgArmsSetTorques=162,
		DbgArmsSetSequences=163,
		DbgArmsExecuteSequence=164,
		DbgArmsGoToPosition=165,
		ArmsStateChange=166,

		DbgRobotSetCommand=176,
		DbgRobotSetPoint=177,
		DbgRobotSetSequence=178,
		DbgRobotExecuteSequence=179,
		DbgRobotSetTrajectoryPoint=180,

		// FPGA
		FpgaGetVersion=256,
		FpgaDbgReadReg=257,
		FpgaDbgWriteReg=258,
		FpgaCmdServo=272,
		FpgaCmdDCMotor=288,
		FpgaCmdPumpR=289,
		FpgaCmdPumpL=290,
		FpgaCmdConveyorBelt=291,
		FpgaCmdStepper=304,
		FpgaGetStepperPos=305,
		FpgaColumnsCalib=320,
		FpgaColumnsMove=321,
		FpgaColumnsSetOffset=322,

		RobotBeginLoadConfig=400,
		RobotLoadConfig=401,
		RobotEndLoadConfig=402,

		RplidarStart=1024,
		RplidarStop=1025,

		RplidarRobotDetection=1280,

	};
}
