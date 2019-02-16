package org.team2168.robot;  

import org.team2168.PID.sensors.AverageEncoder;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.I2C;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around\][\ */
public class RobotMap {

public static final double MAIN_PERIOD_S = 1.0/50.0; // Main loop 200Hz


	/*************************************************************************
	 *                        ROBORIO WIRING MAP                             *
	 *************************************************************************/

	// Joysticks///////////////////////////////////////////////////////////////
	public static final int DRIVER_JOYSTICK = 0;
	public static final int OPERATOR_JOYSTICK = 1;
	public static final int DRIVER_OPERATOR_E_BACKUP = 2;
	public static final int COMMANDS_TEST_JOYSTICK = 4;
	public static final int PID_TEST_JOYSTICK = 5;


	// Joystick Control Styles/////////////////////////////////////////////////
	public static final int TANK_DRIVE_STYLE_ENUM = 0;
	public static final int GUN_STYLE_ENUM = 1;
	public static final int ARCADE_STYLE_ENUM = 2;
	public static final int GTA_STYLE_ENUM = 3;
	public static final int NEW_GUN_STYLE_ENUM = 4;

	// PWM (0 to 9) on RoboRio/////////////////////////////////////////////////
	public static final int RIGHT_DRIVE_MOTOR_1 = 0; //Same as 2017
	public static final int RIGHT_DRIVE_MOTOR_2 = 1; //Same as 2017
	public static final int RIGHT_DRIVE_MOTOR_3 = 2; 
	public static final int LEFT_DRIVE_MOTOR_1 = 3; //Same as 2017
	public static final int LEFT_DRIVE_MOTOR_2 = 4; //Same as 2017
	public static final int LEFT_DRIVE_MOTOR_3 = 5; 

	public static final int LIFT_MOTOR_1 = 6; 
	public static final int LIFT_MOTOR_2 = 7; 

	public static final int PLUNGER_ARM_PIVOT_MOTOR = 8;

	public static final int CARGO_INTAKE_MOTOR = 9;


	// Can Ports ////////////////////////////////////
	public static final int RIGHT_DRIVE_MOTOR_1_CAN = 1; 
	public static final int RIGHT_DRIVE_MOTOR_2_CAN = 2; 
	public static final int RIGHT_DRIVE_MOTOR_3_CAN = 3;
	public static final int LEFT_DRIVE_MOTOR_1_CAN = 4; 
	public static final int LEFT_DRIVE_MOTOR_2_CAN = 5;
	public static final int LEFT_DRIVE_MOTOR_3_CAN = 6;
	
	public static final int LIFT_MOTOR_1_CAN= 7; 
	public static final int LIFT_MOTOR_2_CAN= 8; 
	
	
	public static final int CUBE_INTAKE_MOTOR_LEFT = 9; 
	public static final int CUBE_INTAKE_MOTOR_RIGHT = 10; 
	
  public static final int PLATFORM_MOTOR = 11; 

	public static final int STINGER_WINCH_MOTOR_1 = 12;
	public static final int STRINGER_WINCH_MOTOR_2 = 13;
	
  
  
	//CAN Device IDs///////////////////////////////////////////////////////////
	public static final int PCM_CAN_ID = 0;
	public static final int PCM_CAN_ID_2 = 1;
	public static final int PDP_CAN_ID = 0;
	public static final int TALON_ID = 0;
	public static final int TALON_ID_1 = 1;

	//SPI Channels/////////////////////////////////////////////////////////////
	public static final int GYRO = 0;

	// Digital IO Channels//////////////////////////////////////////////////////
	// Channels 0-9 on RoboRio
	public static final int RIGHT_DRIVE_ENCODER_A = 0; //same as 2017
	public static final int RIGHT_DRIVE_ENCODER_B = 1; //same as 2017
	public static final int LEFT_DRIVE_ENCODER_B = 2; //same as 2017
	public static final int LEFT_DRIVE_ENCODER_A = 3; //same as 2017
	public static final int CUBE_INTAKE_ROTATE_UP_LIMIT = 4;
	public static final int CUBE_INTAKE_ROTATE_DOWN_LIMIT = 5;
	public static final int LIFT_FULLY_UP_LIMIT = 6; //2018 proto
	public static final int LIFT_FULLY_DOWN_LIMIT = 7; //2018 proto
	public static final int LIFT_RATCHET_ENGAGE_LIMIT = 8;
	public static final int LINE_DETECTOR = 9;
	//public static final int TX1_ON_STATUS = 9;


	//Channels 10-25 on MXP (PWM and DIO)
  public static final int PWM_LIGHTS = 18;
	public static final int CAN_DRIVETRAIN_JUMPER = 23; 
	public static final int PRACTICE_BOT_JUMPER = 24;
	

	// PBOT Differences 
	// public static final int GEAR_INTAKE_ARM_HALL_EFECT_PBOT = 0;

	//Solenoid Channels//////////////////////////////////////////

	//Double Soldenoids PCM ID = 0
	public final static int DRIVETRAIN_ENGAGED_PCM = 0;
	
	public static final int HATCH_INTAKE_LOWER_PCM = 2;
	public static final int HATCH_INTAKE_RAISE_PCM = 3;
	public static final int STINGER_RACHET_ENGAGE_PCM = 4;
	public static final int STINGER_RACHET_DISENGAGE_PCM = 5;
	public static final int LIFT_BRAKE_ENGAGE_PCM = 6;
	public static final int LIFE_BRAKE_DISENGAGE = 7;
	
	//Double Soldenoids PCM ID = 1
	public static final int PLUNGER_ARM_BREAK_EXTENDED_PCM = 0;
	public static final int PLUNGER_ARM_BREAK_RETRACTED_PCM = 1;
	public static final int BALL_PUNCH_EXTEND_PCM = 2;
	public static final int BALL_PUNCH_RETRACT_PCM = 3;
	public static final int PROBE_EXTEND_PCM = 4;
	public static final int PROBE_RETRACT_PCM = 5;
	public static final int PROBE_ENGAGE_PCM = 6;
	public static final int PROBE_DISENGAGE_PCM = 7;

	//Analog Input Channels////////////////////////////////////////////////////
	//Channels 0-3 on Roborio
	public static final int LIFT_POSITION_POT = 0;
	public static final int PRESSURE_SENSOR = 4;
	public static final int PIVOT_POSITION_POT = 7; //TODO set input channel (NOT ON ROBOTMAP whiteboard pictures)


	// Channels 4-7 on MXP

	// TODO: Confirm PDP Ports
	// TODO: Should be changed to match the new configuration
	// PDP Channels/////////////////////////////////////////////////////////////

	///////////////////40 Amp////////////////////////////////
	public static final int DRIVETRAIN_RIGHT_MOTOR_1_PDP = 0;
	public static final int DRIVETRAIN_RIGHT_MOTOR_2_PDP = 1;
	public static final int DRIVETRAIN_RIGHT_MOTOR_3_PDP = 2;
	public static final int DRIVETRAIN_LEFT_MOTOR_1_PDP = 15;
	public static final int DRIVETRAIN_LEFT_MOTOR_2_PDP = 14;
	public static final int DRIVETRAIN_LEFT_MOTOR_3_PDP = 13;

	public static final int LIFT_MOTOR_1_PDP = 3;
	public static final int LIFT_MOTOR_2_PDP = 12;
	//public static final int PLATFORM_1_PDP = 3;
	//public static final int PLATFORM_2_PDP = 12;

	// public static final int WINCH_1_PDP = 2;
	// public static final int WINCH_2_PDP = 13;

	///////////20-30 Amp/////////////////////////////////////
	public static final int PLUNGER_ARM_PIVOT_MOTOR_PDP = 4;
	public static final int INTAKE_MOTOR_PDP = 5;


	
	public static final int INTAKE_PIVOT_MOTOR_PDP = 6;
	public static final int COMPRESSOR_PDP = 7;
	public static final int AUX_POWER = 8;

	public static final int PCM_POWER_PCM = 7;;


	// Relay Channels///////////////////////////////////////////////////////////
	public static final int FLASHLIGHT_RELAY = 0;


	/*************************************************************************
	 *                         PBOT DIFFERENCES  PARAMETERS                         *
	 *************************************************************************/


	public static final int LIFT_POSITION_POT_PBOT = 0;
	public static final int PIVOT_POSITION_POT_PBOT = 7; //TODO SET THIS


	/*************************************************************************
	 *                         DRIVETRAIN PARAMETERS                         *
	 *************************************************************************/
	// TODO check if the reverse values match the physical robot
	public static final boolean DT_REVERSE_LEFT1 = false; //false
	public static final boolean DT_REVERSE_LEFT2 = false; //false
	public static final boolean DT_REVERSE_LEFT3 = false; //GUESS 2019
	public static final boolean DT_REVERSE_RIGHT1 = true; //true
	public static final boolean DT_REVERSE_RIGHT2 = true; //true
	public static final boolean DT_REVERSE_RIGHT3 = true; //GUESS 2019

	private static final int DRIVE_PULSE_PER_ROTATION = 256; // encoder ticks per rotation
	// TODO find ratio
	private static final double DRIVE_GEAR_RATIO = 1.0 / 1.0; // ratio between wheel over encoder
	private static final double DRIVE_WHEEL_DIAMETER = 6.0;   //6.0;
	public static final int DRIVE_ENCODER_PULSE_PER_ROT = (int) (DRIVE_PULSE_PER_ROTATION * DRIVE_GEAR_RATIO); // pulse per rotation * gear																					// ratio
	public static final double DRIVE_ENCODER_DIST_PER_TICK = (Math.PI * DRIVE_WHEEL_DIAMETER / DRIVE_ENCODER_PULSE_PER_ROT);
	public static final CounterBase.EncodingType DRIVE_ENCODING_TYPE = CounterBase.EncodingType.k4X; // count rising and falling edges on
	public static final AverageEncoder.PositionReturnType DRIVE_POS_RETURN_TYPE = AverageEncoder.PositionReturnType.FEET;
	public static final AverageEncoder.SpeedReturnType DRIVE_SPEED_RETURN_TYPE = AverageEncoder.SpeedReturnType.FPS;
	public static final int DRIVE_ENCODER_MIN_RATE = 0;
	public static final int DRIVE_ENCODER_MIN_PERIOD = 1;
	public static final boolean LEFT_DRIVE_TRAIN_ENCODER_REVERSE = true;
	public static final boolean RIGHT_DRIVE_TRAIN_ENCODER_REVERSE = true;
	public static final boolean LEFT_DRIVE_TRAIN_ENCODER_REVERSE_PBOT = true;
	public static final boolean RIGHT_DRIVE_TRAIN_ENCODER_REVERSE_PBOT = true;
	public static final int DRIVE_AVG_ENCODER_VAL = 5;
	public static final double MIN_DRIVE_SPEED = 0.2;
	public static final double AUTO_NORMAL_SPEED = 0.5;
	public static final double WHEEL_BASE = 2; //units must match PositionReturnType (feet)

	/*************************************************************************
	 *                         FORKLIFT PARAMETERS                           *
	 *************************************************************************/
	public static final boolean FORKLIFT_LEFT_REVERSE = false;
	public static final boolean FORKLIFT_RIGHT_REVERSE = false;



	/*************************************************************************
	 *                         CUBE INTAKE PARAMETERS                        *
	 *************************************************************************/
	public static final boolean INTAKE_LEFT_REVERSE = true;
	public static final boolean INTAKE_RIGHT_REVERSE = true;
	public static final double CUBE_INTAKE_IR_THRESHOLD = 2.9; //was 2.2\\
	public static final double CUBE_INTAKE_IR_THRESHOLD_PBOT = 2.8;
	public static final boolean INTAKE_PIVOT_REVERSE = false;
	public static final double CUBE_INTAKE_MAX_OUTAKE = -0.8;
	public static final double CUBE_INTAKE_MAX_INTAKE = 1.0;
	public static final double CUBE_PIVOT_CONSTANT_NO_CUBE = 4.0;
	public static final double CUBE_PIVOT_CONSTANT = 4.0;
	public static final double CUBE_PIVOT_DOWN_CONSTANT = 0.6;
	public static final double CUBE_INTAKE_TIMEOUT = 0.4;
	public static final double CUBE_INTAKE_PIVOT_MIN_SPEED = 0.2;
	public static final double CUBE_INTAKE_PIVOT_JOYSTICK_MAX_SPEED = 0.2;
	public static final double CUBE_INTAKE_WHEELS_JOYSTICK_MAX_SPEED = 1.0; 
	public static final double AUTO_CUBE_INTAKE_VALUE = 0.4;

	/*************************************************************************
	 *                         SCISSOR LIFT PARAMETERS                       *
	 *************************************************************************/
	//TODO check if the reverse values match the physical robot
	public static final boolean SCISSOR_LIFT_REVERSE = true;
	/*************************************************************************
	 *                         LIFT PARAMETERS                               *
	 *************************************************************************/
	public static final boolean LIFT_MOTOR1_REVERSE = true;
	public static final boolean LIFT_MOTOR2_REVERSE = true;

	public static final boolean LIFT_MOTOR3_REVERSE = true;
	public static final double LIFT_MAX_JOYSTICK_SPEED = 0.85; 
	public static final double LIFT_UP_MIN_VOLTAGE = 1.2;
	public static final double LIFT_DOWN_MIN_VOLTAGE = 1.2;

	public static final double LIFT_MIN_SPEED = 0.15;


	public static final double LIFT_POT_VOLTAGE_MAX = 4.0; //85 degrees
	public static final double LIFT_POT_MAX_HEIGHT_INCHES = 82.5;
	public static final double LIFT_POT_VOLTAGE_0 = 0.62; //0 degrees
	public static final double LIFT_POT_0_HEIGHT_INCHES = 0.0;

	public static final double LIFT_POT_VOLTAGE_MAX_PBOT = 4.0; //90 degrees
	//public static final double LIFT_POT_VOLTAGE_MAX_PBOT = 0.7; //90 degrees
	public static final double LIFT_POT_MAX_HEIGHT_INCHES_PBOT = 82.5;
	public static final double LIFT_POT_VOLTAGE_0_PBOT = 0.82; //0 degrees
	//public static final double LIFT_POT_VOLTAGE_0_PBOT = 4.46; //0 degrees
	public static final double LIFT_POT_0_HEIGHT_INCHES_PBOT = 0.0;


	public static final boolean ENABLE_LIFT_POT_SAFETY = true;
	public static final int LIFT_AVG_ENCODER_VAL = 5;

	/*************************************************************************
	 *                         Winch PARAMETERS                               *
	 *************************************************************************/

	public static final boolean WINCH_MOTOR1_REVERSE = true;
	public static final boolean WINCH_MOTOR2_REVERSE = true;

	/*************************************************************************
	 *                         Plunger Arm Pivot PARAMETERS                   *
	 *************************************************************************/
		/////TODO set these values
	public static final double PIVOT_POT_VOLTAGE_0 = 0;
	public static final double PIVOT_POT_VOLTAGE_MAX = 5;
	public static final double PIVOT_POT_0_ROTATION_DEGREES = 0;
	public static final double PIVOT_POT_MAX_ROTATION_DEGREES = 180;

	/////TODO set these values
	public static final double PIVOT_POT_VOLTAGE_0_PBOT = 0;
	public static final double PIVOT_POT_VOLTAGE_MAX_PBOT = 5;
	public static final double PIVOT_POT_0_ROTATION_DEGREES_PBOT = 0;
	public static final double PIVOT_POT_MAX_ROTATION_DEGREES_PBOT = 180;

	public static final int PIVOT_AVG_ENCODER_VAL = 5; //taken from 2018 lift encoder

	public static final boolean PLUNGER_ARM_PIVOT_REVERSE = false; //TODO set

	/*************************************************************************
	 *                         PDP PARAMETERS                                *
	 *************************************************************************/
	public static final long PDPThreadPeriod = 100;
	public static final double WARNING_CURRENT_LIMIT = 20;
	public static final double STALL_CURRENT_LIMIT = 35;
	public static final double CURRENT_LIMIT_TIME_THRESHOLD_SECONDS = 1;

	/*************************************************************************
	 *                         PID PARAMETERS                                *
	 *************************************************************************/
	// period to run PID loops on drive train
	public static final long DRIVE_TRAIN_PID_PERIOD = 20;// 70ms loop
	public static final int DRIVE_TRAIN_PID_ARRAY_SIZE = 30;

	public static final double DRIVE_TRAIN_MIN_FWD_VOLTAGE = 1.8;//volts
	public static final double DRIVE_TRAIN_MIN_RVD_VOLTAGE = 1.2;//volts

	public static final double DRIVE_TRAIN_MIN_ROT_CLOCKWISE_VOLTAGE = 1.45;//volts
	public static final double DRIVE_TRAIN_MIN_ROT_COUNTCLOCKWISE_VOLTAGE = 1.45;//volts

	// PID Gains for Left & Right Speed and Position
	// Bandwidth =
	// Phase Margin =
	public static final double DRIVE_TRAIN_LEFT_SPEED_P = 0.04779;
	public static final double DRIVE_TRAIN_LEFT_SPEED_I = 0.0010526;
	public static final double DRIVE_TRAIN_LEFT_SPEED_D = 0.0543;

	public static final double DRIVE_TRAIN_RIGHT_SPEED_P = 0.04779;
	public static final double DRIVE_TRAIN_RIGHT_SPEED_I = 0.0010526;
	public static final double DRIVE_TRAIN_RIGHT_SPEED_D = 0.0543;

	public static final double DRIVE_TRAIN_LEFT_POSITION_P = 0.2;
	public static final double DRIVE_TRAIN_LEFT_POSITION_I = 0.0001412646174233;
	public static final double DRIVE_TRAIN_LEFT_POSITION_D = 0.0074778888124088;

	public static final double DRIVE_TRAIN_RIGHT_POSITION_P = 0.25;
	public static final double DRIVE_TRAIN_RIGHT_POSITION_I = 0.0001412646174233;
	public static final double DRIVE_TRAIN_RIGHT_POSITION_D = 0.0074778888124088;

	public static final double ROTATE_POSITION_P = 0.055;
	public static final double ROTATE_POSITION_I = 0.001;
	public static final double ROTATE_POSITION_D = 0.0064778888124088;

	public static final double ROTATE_POSITION_P_Drive_Straight = 0.055; //0.055 comp
	public static final double ROTATE_POSITION_I_Drive_Straight = 0.001; //0.001
	public static final double ROTATE_POSITION_D_Drive_Straight = 0.0064778888124088;



	public static final double LIFT_P = 0.024;
	public static final double LIFT_I = 0.027;
	public static final double LIFT_D = 0.000000067;
	public static final double LIFT_N = 100;

	// Turret Period
	public static final long LIFT_PID_PERIOD = 20;
	public static final int  LIFT_PID_ARRAY_SIZE = 30;

	/****************************************************************
	 *                         TCP Servers (ONLY FOR DEBUGGING)     *
	 ****************************************************************/
	public static final int TCP_SERVER_DRIVE_TRAIN_POS = 1180;
	public static final int TCP_SERVER_ROTATE_CONTROLLER = 1181;
	public static final int TCO_SERVER_RIGHT_DRIVE_TRAIN_SPEED = 1182;
	public static final int TCP_SERVER_LEFT_DRIVE_TRAIN_SPEED = 1183;
	public static final int TCP_SERVER_LIFT_POT_CONTROLLER = 1184;
	public static final int TCP_SERVER_ROTATE_CONTROLLER_STRAIGHT = 1185;


	/******************************************************************
	 *                         ConsolePrinter PARAMETERS              *
	 ******************************************************************/
	public static final boolean PRINT_SD_DEBUG_DATA = true;
	public static final long SmartDashThreadPeriod = 100; // ms
	public static final long CONSOLE_PRINTER_LOG_RATE_MS = 100; // ms

	/******************************************************************
	 *                         Lights I2C                             *
	 ******************************************************************/
	public static final I2C.Port I2C_PORT = I2C.Port.kOnboard;
	public static final int I2C_ADDRESS = 10;

	/******************************************************************
	 *                        Kevin PARAMETERS                        *
	 ******************************************************************/
	public static final boolean KEVIN_IS_DA_BOMB = true;
	public static final boolean GUYANA_HAS_SUNK = false; //debatable

	/******************************************************************
	 *                        Aiden Parameters                         *
	 ********************************************`**********************/

	public static final String DID_AIDEN_PUSH_IT = "only if Liam said he could"; //so false
	public static final boolean DID_AIDEN_WRITE_A_PATH = false;




}