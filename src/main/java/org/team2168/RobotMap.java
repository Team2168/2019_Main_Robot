package org.team2168;  

import com.sun.org.apache.bcel.internal.generic.I2C;

import org.team2168.PID.sensors.AverageEncoder;

import edu.wpi.first.wpilibj.CounterBase;

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
	public static final int RIGHT_DRIVE_MOTOR_1 = 0; 
	public static final int RIGHT_DRIVE_MOTOR_2 = 1; 
	public static final int RIGHT_DRIVE_MOTOR_3 = 2; 
	public static final int LEFT_DRIVE_MOTOR_1 = 3; 
	public static final int LEFT_DRIVE_MOTOR_2 = 4; 
	public static final int LEFT_DRIVE_MOTOR_3 = 5; 


	// Digital IO Channels//////////////////////////////////////////////////////
	// Channels 0-9 on RoboRio
	public static final int RIGHT_DRIVE_ENCODER_A = 0;
	public static final int RIGHT_DRIVE_ENCODER_B = 1;
	public static final int LEFT_DRIVE_ENCODER_B = 2;
	public static final int LEFT_DRIVE_ENCODER_A = 3;

	public static final int LIFT_FULLY_UP_LIMIT = 4;
	public static final int LIFT_FULLY_DOWN_LIMIT = 5;

	public static final int STINGER_LEFT_RATCHET_ENGAGED = 6;
	public static final int STINGER_RIGHT_RATCHET_ENGAGED = 7;

	public static final int HATCH_INTAKE_LOWERED_LIMIT = 8;
	public static final int HATCH_INTAKE_RAISED_LIMIT = 9;


	//Channels 10-25 on MXP (PWM and DIO)
  	public static final int PWM_LIGHTS = 18;
	public static final int CAN_DRIVETRAIN_JUMPER = 23; 
	public static final int PRACTICE_BOT_JUMPER = 24;

	//Analog Input Channels////////////////////////////////////////////////////
	//Channels 0-3 on Roborio
	public static final int HATCH_INTAKE_IR_SENSOR = 0;
	public static final int LIFT_POSITION_POT = 1;
	public static final int CARGO_INTAKE_SHARP_IR_SENSOR = 2;
	public static final int PRESSURE_SENSOR = 3;

	public static final int MONKEY_BAR_AVERAGE_POTENTIOMETER_LEFT = 5;
	public static final int MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT = 6;
	public static final int PIVOT_POSITION_POT = 7;

	// Channels 4-7 on MXP
	

	/*************************************************************************
	*                         CAN DEVICES                                    *
	*************************************************************************/
	
	//CAN Device IDs///////////////////////////////////////////////////////////
	public static final int PCM_CAN_ID_BELLYPAN = 0;
	public static final int PCM_CAN_ID_LIFT = 1;
	public static final int PDP_CAN_ID = 0;

	/*************************************************************************
	*                         SPI DEVICES                                    *
	*************************************************************************/
	public static final int GYRO = 0;


	/*************************************************************************
	*                         Solenoids                                      *
	*************************************************************************/

	//Double Soldenoids PCM ID = 0
	public final static int DRIVETRAIN_ENGAGED_PCM = 0;
	public static final int HATCH_PANEL_VELCRO_PUSHER_PCM = 1;
	public static final int HATCH_INTAKE_LOWER_PCM = 2;
	public static final int HATCH_INTAKE_RAISE_PCM = 3;
	public static final int STINGER_RACHET_ENGAGE_PCM = 4;
	public static final int STINGER_RACHET_DISENGAGE_PCM = 5;
	public static final int LIFT_BRAKE_ENGAGE_PCM = 6;
	public static final int LIFT_BRAKE_DISENGAGE_PCM = 7;
	
	//Double Soldenoids PCM ID = 1
	public static final int PROBE_ROTATE_BRAKE_EXTENDED_PCM = 0;
	public static final int PROBE_ROTATE_BRAKE_RETRACTED_PCM = 1;
	public static final int BALL_PUNCH_EXTEND_PCM = 2;
	public static final int BALL_PUNCH_RETRACT_PCM = 3;
	public static final int PROBE_EXTEND_PCM = 4;
	public static final int PROBE_RETRACT_PCM = 5;
	public static final int PROBE_ENGAGE_PCM = 6;
	public static final int PROBE_DISENGAGE_PCM = 7;



	/*************************************************************************
	*                         PDP/CAN DEVICES                                 *
	*************************************************************************/
	public static final int LIFT_MOTOR_1_PDP = 0;
	public static final int DRIVETRAIN_RIGHT_MOTOR_1_PDP = 1;
	public static final int DRIVETRAIN_LEFT_MOTOR_1_PDP = 2;
	public static final int LIFT_MOTOR_2_PDP = 3;
	public static final int MONKEY_BAR_INTAKE_WHEELS_LEFT_PDP = 4;
	public static final int MONKEY_BAR_INTAKE_WHEELS_RIGHT_PDP = 5;
	public static final int HATCH_FLOOR_INTAKE_PDP= 6;
	public static final int COMPRESSOR_PDP = 7;
	public static final int CARGO_INTAKE_MOTOR_PDP = 8;
	public static final int PLUNGER_PIVOT_MOTOR_PDP = 9;
	public static final int MONKEY_BAR_ROTATE_LEFT_PDP = 10;
	public static final int MONKEY_BAR_ROTATE_RIGHT_PDP = 11;


	public static final int DRIVETRAIN_LEFT_MOTOR_2_PDP = 14;
	public static final int DRIVETRAIN_RIGHT_MOTOR_2_PDP = 15;
	public static final int DRIVETRAIN_RIGHT_MOTOR_3_PDP = 30; //not used in 4 motor DT
	public static final int DRIVETRAIN_LEFT_MOTOR_3_PDP = 31; //not used in 4 motor DT


	// Relay Channels///////////////////////////////////////////////////////////
	public static final int FLASHLIGHT_RELAY = 0;


	/*************************************************************************
	 *                         PBOT DIFFERENCES  PARAMETERS                  *
	 *************************************************************************/


	public static final int LIFT_POSITION_POT_PBOT = 0;
	public static final int PIVOT_POSITION_POT_PBOT = 7; //TODO SET THIS
	public static final boolean LEFT_DRIVE_TRAIN_ENCODER_REVERSE_PBOT = true;
	public static final boolean RIGHT_DRIVE_TRAIN_ENCODER_REVERSE_PBOT = true;


	/*************************************************************************
	 *                         DRIVETRAIN PARAMETERS                         *
	 *************************************************************************/
	// TODO check if the reverse values match the physical robot
	public static final boolean DT_REVERSE_LEFT1 = false;
	public static final boolean DT_REVERSE_LEFT2 = false;
	public static final boolean DT_REVERSE_LEFT3 = false;
	public static final boolean DT_REVERSE_RIGHT1 = true;
	public static final boolean DT_REVERSE_RIGHT2 = true;
	public static final boolean DT_REVERSE_RIGHT3 = true; 

	public static final boolean DT_3_MOTORS_PER_SIDE = false;

	private static final int DRIVE_PULSE_PER_ROTATION = 256; // encoder ticks per rotation

	private static final double DRIVE_GEAR_RATIO = 1.0 / 1.0; // ratio between wheel over encoder
	private static final double DRIVE_WHEEL_DIAMETER = 6.0;   //inches;
	public static final int DRIVE_ENCODER_PULSE_PER_ROT = (int) (DRIVE_PULSE_PER_ROTATION * DRIVE_GEAR_RATIO); // pulse per rotation * gear																					// ratio
	public static final double DRIVE_ENCODER_DIST_PER_TICK = (Math.PI * DRIVE_WHEEL_DIAMETER / DRIVE_ENCODER_PULSE_PER_ROT);
	public static final CounterBase.EncodingType DRIVE_ENCODING_TYPE = CounterBase.EncodingType.k4X; // count rising and falling edges on
	public static final AverageEncoder.PositionReturnType DRIVE_POS_RETURN_TYPE = AverageEncoder.PositionReturnType.INCH;
	public static final AverageEncoder.SpeedReturnType DRIVE_SPEED_RETURN_TYPE = AverageEncoder.SpeedReturnType.IPS;
	public static final int DRIVE_ENCODER_MIN_RATE = 0;
	public static final int DRIVE_ENCODER_MIN_PERIOD = 1;
	public static final boolean LEFT_DRIVE_TRAIN_ENCODER_REVERSE = true;
	public static final boolean RIGHT_DRIVE_TRAIN_ENCODER_REVERSE = true;

	public static final int DRIVE_AVG_ENCODER_VAL = 5;
	public static final double MIN_DRIVE_SPEED = 0.2;
	public static final double AUTO_NORMAL_SPEED = 0.5;
	public static final double WHEEL_BASE = 26; //units must match PositionReturnType (inch)




	/*************************************************************************
	 *                         CARGO INTAKE PARAMETERS                        *
	 *************************************************************************/
	public static final boolean CARGO_INTAKE_MOTOR_REVERSE = true;
	public static final double CARGO_INTAKE_IR_THRESHOLD_MIN = 0.0; 
	public static final double CARGO_INTAKE_IR_THRESHOLD_MAX = 0.0; 

	public static final double CARGO_INTAKE_IR_THRESHOLD_MIN_PBOT = 0.0; 
	public static final double CARGO_INTAKE_IR_THRESHOLD_MAX_PBOT = 0.0; 

	/*************************************************************************
	 *                         HATCH INTAKE PARAMETERS                        *
	 *************************************************************************/
	public static final boolean HATCH_INTAKE_MOTOR_REVERSE = true;
	public static final double HATCH_INTAKE_IR_THRESHOLD_MIN = 0.0; 
	public static final double HATCH_INTAKE_IR_THRESHOLD_MAX = 0.0; 

	public static final double HATCH_INTAKE_IR_THRESHOLD_MIN_PBOT = 0.0; 
	public static final double HATCH_INTAKE_IR_THRESHOLD_MAX_PBOT = 0.0; 

	/*************************************************************************
	 *                         LIFT PARAMETERS                               *
	 *************************************************************************/
	public static final boolean LIFT_MOTOR1_REVERSE = false;
	public static final boolean LIFT_MOTOR2_REVERSE = false;

	public static final double LIFT_MAX_JOYSTICK_SPEED = 0.85; 

	public static final double LIFT_UP_MIN_VOLTAGE = 1.2;
	public static final double LIFT_DOWN_MIN_VOLTAGE = 1.2;
	public static final double LIFT_MIN_SPEED = 0.15;


	public static final double LIFT_POT_VOLTAGE_MAX = 4.0; //85 degrees
	public static final double LIFT_POT_MAX_HEIGHT_INCHES = 82.5;
	public static final double LIFT_POT_VOLTAGE_0 = 0.62; //0 degrees
	public static final double LIFT_POT_0_HEIGHT_INCHES = 0.0;

	//PBOT LIFT
	public static final double LIFT_POT_VOLTAGE_MAX_PBOT = 4.0; //90 degrees
	public static final double LIFT_POT_MAX_HEIGHT_INCHES_PBOT = 82.5;
	public static final double LIFT_POT_VOLTAGE_0_PBOT = 0.82; //0 degrees
	public static final double LIFT_POT_0_HEIGHT_INCHES_PBOT = 0.0;


	public static final boolean ENABLE_LIFT_POT_SAFETY = true;
	public static final int LIFT_AVG_ENCODER_VAL = 5;


	/*************************************************************************
	 *                      Monkey Bar PARAMETERS                            *
	 *************************************************************************/
	public static final boolean MONKEY_BAR_ROTATE_RIGHT_REVERSE = false;
	public static final boolean MONKEY_BAR_ROTATE_LEFT_REVERSE = false;
	
	public static final boolean MONKEY_BAR_INTAKE_RIGHT_REVERSE = false;
	public static final boolean MONKEY_BAR_INTAKE_LEFT_REVERSE = false;

	public static final double MONKEY_BAR_LEFT_POT_VOLTAGE_MAX = 5.108; //130 degrees
	public static final double MONKEY_BAR_LEFT_POT_MAX_ROTATION = 130; //130 degrees
	public static final double MONKEY_BAR_LEFT_POT_VOLTAGE_0 = 0.66; //0 degrees
	public static final double MONKEY_BAR_LEFT_ANGLE_DEGREES_0 = 0.0;
	
	public static final double MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX = 5.108; //130 degrees
	public static final double MONKEY_BAR_RIGHT_POT_MAX_ROTATION = 130; //130 degrees
	public static final double MONKEY_BAR_RIGHT_POT_VOLTAGE_0 = 0.66; //0 degrees
	public static final double MONKEY_BAR_RIGHT_ANGLE_DEGREES_0 = 0.0;
	public static final int MONKEY_BAR_RIGHT_AVG_ENCODER_VAL = 5;


	//PBOT
	public static final double MONKEY_BAR_LEFT_POT_VOLTAGE_MAX_PBOT = 5.108; //130 degrees
	public static final double MONKEY_BAR_LEFT_POT_MAX_ROTATION_PBOT = 130; //130 degrees
	public static final double MONKEY_BAR_LEFT_POT_VOLTAGE_0_PBOT = 0.66; //0 degrees
	public static final double MONKEY_BAR_LEFT_ANGLE_DEGREES_0_PBOT = 0.0;
	
	public static final double MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX_PBOT = 5.108; //130 degrees
	public static final double MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT = 130; //130 degrees
	public static final double MONKEY_BAR_RIGHT_POT_VOLTAGE_0_PBOT = 0.66; //0 degrees
	public static final double MONKEY_BAR_RIGHT_ANGLE_DEGREES_0_PBOT = 0.0;
	public static final int MONKEY_BAR_RIGHT_AVG_ENCODER_VAL_PBOT = 5;


	public static final int MONKEY_BAR_AVG_ENCODER_VAL = 5;

	/*************************************************************************
	 *                         Stinger Winch PARAMETERS                               *
	 *************************************************************************/

	public static final boolean WINCH_MOTOR_LEFT_REVERSE = true;
	public static final boolean WINCH_MOTOR_RIGHT_REVERSE = true;

	public static final double STINGER_LEFT_POT_VOLTAGE_MAX = 4.0; //85 degrees
	public static final double STINGER_LEFT_POT_MAX_HEIGHT_INCHES = 82.5;
	public static final double STINGER_LEFT_POT_VOLTAGE_0 = 0.62; //0 degrees
	public static final double STINGER_LEFT_POT_0_HEIGHT_INCHES = 0.0;

	public static final double STINGER_RIGHT_POT_VOLTAGE_MAX = 4.0; //85 degrees
	public static final double STINGER_RIGHT_POT_MAX_HEIGHT_INCHES = 82.5;
	public static final double STINGER_RIGHT_POT_VOLTAGE_0 = 0.62; //0 degrees
	public static final double STINGER_RIGHT_POT_0_HEIGHT_INCHES = 0.0;

	//PBOT
	public static final double STINGER_LEFT_POT_VOLTAGE_MAX_PBOT = 4.0; //85 degrees
	public static final double STINGER_LEFT_POT_MAX_HEIGHT_INCHES_PBOT = 82.5;
	public static final double STINGER_LEFT_POT_VOLTAGE_0_PBOT = 0.62; //0 degrees
	public static final double STINGER_LEFT_POT_0_HEIGHT_INCHES_PBOT = 0.0;

	public static final double STINGER_RIGHT_POT_VOLTAGE_MAX_PBOT = 4.0; //85 degrees
	public static final double STINGER_RIGHT_POT_MAX_HEIGHT_INCHES_PBOT = 82.5;
	public static final double STINGER_RIGHT_POT_VOLTAGE_0_PBOT = 0.62; //0 degrees
	public static final double STINGER_RIGHT_POT_0_HEIGHT_INCHES_PBOT = 0.0;

	public static final int STINGER_AVG_ENCODER_VAL = 5;
	/*************************************************************************
	 *                         Plunger Arm Pivot PARAMETERS                   *
	 *************************************************************************/
	////TODO set these values
	public static final boolean PLUNGER_ARM_PIVOT_REVERSE = false; //TODO set
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
	public static final int TCP_SERVER_RIGHT_DRIVE_TRAIN_POSITION = 1186;
	public static final int TCP_SERVER_LEFT_DRIVE_TRAIN_POSITION = 1187;

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