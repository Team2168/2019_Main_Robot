package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.controllers.PIDSpeed;
import org.team2168.PID.sensors.ADXRS453Gyro;
import org.team2168.PID.sensors.AverageEncoder;
import org.team2168.PID.sensors.IMU;
import org.team2168.commands.drivetrain.DriveWithJoystick;
import org.team2168.robot.Robot;
import org.team2168.robot.RobotMap;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Subsystem class for the Drivetrain
 * 
 * @author Cierra O'Grady
 */
public class Drivetrain extends Subsystem {

	private static SpeedController leftMotor1;
	private static SpeedController leftMotor2;
	private static SpeedController leftMotor3;
	private static SpeedController rightMotor1;
	private static SpeedController rightMotor2;
	private static SpeedController rightMotor3;

	private static boolean INVERT_LINE_SENSOR = true; //Line sensor uses negative logic (false = detected)
	
	private ADXRS453Gyro gyroSPI;
	private AverageEncoder drivetrainLeftEncoder;
	private AverageEncoder drivetrainRightEncoder;

	private static AnalogInput DrivetrainSonarSensor;
	private static DigitalInput lineDetector;
	
	public IMU imu;
	

	// declare position/speed controllers
	public PIDPosition driveTrainPosController;
	public PIDPosition rotateController;
	public PIDPosition rotateDriveStraightController;
	

	// declare speed controllers
	public PIDSpeed rightSpeedController;
	public PIDSpeed leftSpeedController;

	private static Drivetrain instance = null;

	// declare TCP severs...ONLY FOR DEBUGGING PURPOSES, SHOULD BE REMOVED FOR
	// COMPITITION
	TCPSocketSender TCPdrivePosController;
	TCPSocketSender TCPrightSpeedController;
	TCPSocketSender TCPleftSpeedController;
	TCPSocketSender TCProtateController;

	public volatile double leftMotor1Voltage;
	public volatile double leftMotor2Voltage;
	public volatile double rightMotor1Voltage;
	public volatile double rightMotor2Voltage;

	/**
	 * Default constructors for Drivetrain
	 */
	private Drivetrain() {
		if (Robot.isPracticeRobot())
		{
		  leftMotor1 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_1);
		  leftMotor2 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_2);
		  leftMotor3 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_3);
		  rightMotor1 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_1);
		  rightMotor2 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_2);
		  rightMotor3 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_3);
  
		}
		else if (Robot.isCanRobot())
		{
		  leftMotor1 = new CANSparkMax(RobotMap.DT_MAX_CAN_ID_LEFT_1, MotorType.kBrushless);
		  leftMotor2 = new CANSparkMax(RobotMap.DT_MAX_CAN_ID_LEFT_2,MotorType.kBrushless);
		  leftMotor3 = new CANSparkMax(RobotMap.DT_MAX_CAN_ID_LEFT_3,MotorType.kBrushless);
		  rightMotor1 = new CANSparkMax(RobotMap.DT_MAX_CAN_ID_RIGHT_1,MotorType.kBrushless);
		  rightMotor2 = new CANSparkMax(RobotMap.DT_MAX_CAN_ID_RIGHT_2,MotorType.kBrushless);
		  rightMotor3 = new CANSparkMax(RobotMap.DT_MAX_CAN_ID_RIGHT_3,MotorType.kBrushless);
		}
		else 
		{
		  leftMotor1 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_1);
		  leftMotor2 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_2);
		  leftMotor3 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_3);
		  rightMotor1 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_1);
		  rightMotor2 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_2);
		  rightMotor3 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_3);
  }
		
		drivetrainRightEncoder = new AverageEncoder(
				RobotMap.RIGHT_DRIVE_ENCODER_A,
				RobotMap.RIGHT_DRIVE_ENCODER_B,
				RobotMap.DRIVE_ENCODER_PULSE_PER_ROT,
				RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
				RobotMap.RIGHT_DRIVE_TRAIN_ENCODER_REVERSE,
				RobotMap.DRIVE_ENCODING_TYPE,
				RobotMap.DRIVE_SPEED_RETURN_TYPE,
				RobotMap.DRIVE_POS_RETURN_TYPE,
				RobotMap.DRIVE_AVG_ENCODER_VAL);


		drivetrainLeftEncoder = new AverageEncoder(
				RobotMap.LEFT_DRIVE_ENCODER_A, 
				RobotMap.LEFT_DRIVE_ENCODER_B,
				RobotMap.DRIVE_ENCODER_PULSE_PER_ROT, 
				RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
				RobotMap.LEFT_DRIVE_TRAIN_ENCODER_REVERSE, 
				RobotMap.DRIVE_ENCODING_TYPE,
				RobotMap.DRIVE_SPEED_RETURN_TYPE, 
				RobotMap.DRIVE_POS_RETURN_TYPE, 
				RobotMap.DRIVE_AVG_ENCODER_VAL);

		gyroSPI = new ADXRS453Gyro();
		gyroSPI.startThread();

		imu = new IMU(drivetrainLeftEncoder, drivetrainRightEncoder, RobotMap.WHEEL_BASE);

		DrivetrainSonarSensor = new AnalogInput(RobotMap.DRIVETRAIN_SONAR_SENSOR);


		// DriveStraight Controller
		rotateController = new PIDPosition(
				"RotationController", 
				RobotMap.ROTATE_POSITION_P, 
				RobotMap.ROTATE_POSITION_I,
				RobotMap.ROTATE_POSITION_D, 
				gyroSPI, 
				RobotMap.DRIVE_TRAIN_PID_PERIOD);

		
		rotateDriveStraightController = new PIDPosition(
				"RotationStraightController",
				RobotMap.ROTATE_POSITION_P_Drive_Straight, 
				RobotMap.ROTATE_POSITION_I_Drive_Straight,
				RobotMap.ROTATE_POSITION_D_Drive_Straight, 
				gyroSPI, 
				RobotMap.DRIVE_TRAIN_PID_PERIOD);

		driveTrainPosController = new PIDPosition(
				"driveTrainPosController", 
				RobotMap.DRIVE_TRAIN_RIGHT_POSITION_P,
				RobotMap.DRIVE_TRAIN_RIGHT_POSITION_I, 
				RobotMap.DRIVE_TRAIN_RIGHT_POSITION_D, 
				imu,
				RobotMap.DRIVE_TRAIN_PID_PERIOD);

		// Spawn new PID Controller
		rightSpeedController = new PIDSpeed(
				"rightSpeedController", 
				RobotMap.DRIVE_TRAIN_RIGHT_SPEED_P,
				RobotMap.DRIVE_TRAIN_RIGHT_SPEED_I, 
				RobotMap.DRIVE_TRAIN_RIGHT_SPEED_D, 
				1, 
				drivetrainRightEncoder,
				RobotMap.DRIVE_TRAIN_PID_PERIOD);

		leftSpeedController = new PIDSpeed(
				"leftSpeedController", 
				RobotMap.DRIVE_TRAIN_LEFT_SPEED_P,
				RobotMap.DRIVE_TRAIN_LEFT_SPEED_I, 
				RobotMap.DRIVE_TRAIN_LEFT_SPEED_D, 
				1, 
				drivetrainLeftEncoder,
				RobotMap.DRIVE_TRAIN_PID_PERIOD);

		// add min and max output defaults and set array size
		rightSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
		leftSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
		driveTrainPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
		rotateController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
		rotateDriveStraightController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);


		// start controller threads
		rightSpeedController.startThread();
		leftSpeedController.startThread();
		driveTrainPosController.startThread();
		rotateController.startThread();
		rotateDriveStraightController.startThread();

		// start TCP Servers for DEBUGING ONLY
		TCPdrivePosController = new TCPSocketSender(RobotMap.TCP_SERVER_DRIVE_TRAIN_POS, driveTrainPosController);
		TCPdrivePosController.start();

		TCPrightSpeedController = new TCPSocketSender(RobotMap.TCO_SERVER_RIGHT_DRIVE_TRAIN_SPEED,
				rightSpeedController);
		TCPrightSpeedController.start();

		TCPleftSpeedController = new TCPSocketSender(RobotMap.TCP_SERVER_LEFT_DRIVE_TRAIN_SPEED, leftSpeedController);
		TCPleftSpeedController.start();

		TCProtateController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER, rotateController);
		TCProtateController.start();

		TCProtateController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER_STRAIGHT,
				rotateDriveStraightController);
		TCProtateController.start();


		leftMotor1Voltage = 0;
		leftMotor2Voltage = 0;

		rightMotor1Voltage = 0;
		rightMotor2Voltage = 0;
		
	

		// Log sensor data
		
		//PID Testing order
		
		
		ConsolePrinter.putNumber("Left Encoder Distance", () -> {return Robot.drivetrain.getLeftPosition();}, true, true);
		ConsolePrinter.putNumber("Right Encoder Distance:", () -> {return Robot.drivetrain.getRightPosition();}, true, true);
		ConsolePrinter.putNumber("Average Drive Encoder Distance", () -> {return Robot.drivetrain.getAverageDistance();}, true, true);
		ConsolePrinter.putNumber("Right Drive Encoder Rate", () -> {return Robot.drivetrain.getRightEncoderRate();}, true, true);
		ConsolePrinter.putNumber("Left Drive Encoder Rate", () -> {return Robot.drivetrain.getLeftEncoderRate();}, true, true);
		ConsolePrinter.putNumber("Average Drive Encoder Rate", () -> {return Robot.drivetrain.getAverageEncoderRate();}, true, true);
		ConsolePrinter.putNumber("Gyro Angle:", () -> {return Robot.drivetrain.getHeading();}, true, true);	
		ConsolePrinter.putNumber("Gunstyle X Value", () -> {return Robot.oi.getGunStyleXValue();}, true, true);
		ConsolePrinter.putNumber("Gunstyle Y Value", () -> {return Robot.oi.getGunStyleYValue();}, true, true);
		ConsolePrinter.putNumber("DTLeft1MotorVoltage", () -> {return Robot.drivetrain.getleftMotor1Voltage();}, true, true);
		ConsolePrinter.putNumber("DTLeft2MotorVoltage", () -> {return Robot.drivetrain.getleftMotor2Voltage();}, true, true);
		ConsolePrinter.putNumber("DTRight1MotorVoltage", () -> {return Robot.drivetrain.getrightMotor1Voltage();}, true, true);
		ConsolePrinter.putNumber("DTRight2MotorVoltage", () -> {return Robot.drivetrain.getrightMotor2Voltage();}, true, true);
		ConsolePrinter.putNumber("DTRight1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);}, true, true);
		ConsolePrinter.putNumber("DTRight2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);}, true, true);
		ConsolePrinter.putNumber("DTLeft1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);}, true, true);
		ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, true);
		ConsolePrinter.putNumber("PID right motor 1 voltage", () -> {return this.PIDVoltagefeedRightMotor1();}, true, true);
		ConsolePrinter.putNumber("PID right motor 2 voltage", () -> {return this.PIDVoltagefeedRightMotor2();}, true, true);
		ConsolePrinter.putNumber("PID left motor 1 voltage", () -> {return this.PIDVoltagefeedLeftMotor1();}, true, true);
		ConsolePrinter.putNumber("PID left motor 2 voltage", () -> {return this.PIDVoltagefeedLeftMotor2();}, true, true);
		
		ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, true);
		ConsolePrinter.putNumber("GYRO Driftrate:", () -> {return Robot.drivetrain.gyroSPI.driftRate;}, true, false);
		ConsolePrinter.putNumber("GYRO Rate:", () -> {return Robot.drivetrain.gyroSPI.getRate();}, true, false);
		ConsolePrinter.putNumber("GYRO Angle SPI:", () -> {return Robot.drivetrain.gyroSPI.getAngle();}, true, false);
		ConsolePrinter.putNumber("GYRO reInits:", () -> {return (double) Robot.gyroReinits;}, true, false);
		ConsolePrinter.putBoolean("Gyro Cal Status", () -> {return !Robot.gyroCalibrating;}, true, false);
		ConsolePrinter.putNumber("GYRO Status:", () -> {return (double) Robot.drivetrain.gyroSPI.getStatus();}, true, false);
		ConsolePrinter.putNumber("GYRO Temp:", () -> {return Robot.drivetrain.gyroSPI.getTemp();}, true, false);
		
		
		ConsolePrinter.putBoolean("Left Motor One Trip", () -> {return !Robot.pdp.isLeftMotorOneTrip();}, true, false);
		ConsolePrinter.putBoolean("Left Motor Two Trip", () -> {return !Robot.pdp.isLeftMotorTwoTrip();}, true, false);
		ConsolePrinter.putBoolean("Right Motor One Trip", () -> {return !Robot.pdp.isRightMotorOneTrip();}, true, false);
		ConsolePrinter.putBoolean("Right Motor Two Trip", () -> {return !Robot.pdp.isRightMotorTwoTrip();}, true, false);
		
		ConsolePrinter.putBoolean("Is line detected?", () -> {return getLinedectorStatus();}, true, false);
		
		ConsolePrinter.putNumber("Right Motor One Command", () -> {return rightMotor1.get();}, true, true);
		ConsolePrinter.putNumber("Right Motor Two Command", () -> {return rightMotor2.get();}, true, true);
		
		ConsolePrinter.putNumber("Left Motor One Command", () -> {return leftMotor1.get();}, true, true);
		ConsolePrinter.putNumber("Left Motor Two Command", () -> {return leftMotor2.get();}, true, true);
		
		ConsolePrinter.putNumber("Drivetrain raw sonar", () -> {return Robot.drivetrain.getSonarVoltage();}, true, false);
	}

	/**
	 * Calls instance object and makes it a singleton object of type Drivetrain
	 * 
	 * @returns Drivetrain object "instance"
	 */
	public static Drivetrain getInstance() {
		if (instance == null)
			instance = new Drivetrain();

		return instance;
	}

	
	
	
	
	
	/**
	 * Calls left motor 1 and creates a local variable "speed" Refers to boolean in
	 * Robot map and if true, speed = - speed Uses set() command to assign the new
	 * speed to left motor 1
	 * 
	 * @param double
	 *            speed between -1 and 1 negative is reverse, positive if forward, 0
	 *            is stationary
	 */
	private void driveleftMotor1(double speed) {
		if (RobotMap.DT_REVERSE_LEFT1)
			speed = -speed;

		leftMotor1.set(speed);
		leftMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;

	}

	/**
	 * Calls left motor 2 and creates a local variable "speed" Refers to boolean in
	 * Robot map and if true, speed = - speed Uses set() command to assign the new
	 * speed to left motor 2
	 * 
	 * @param double
	 *            speed between -1 and 1 negative is reverse, positive if forward, 0
	 *            is stationary
	 */
	private void driveleftMotor2(double speed) {
		if (RobotMap.DT_REVERSE_LEFT2)
			speed = -speed;

		leftMotor2.set(speed);
		leftMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
	}

	/**
	 * Take in double speed and sets it to left motors 1, 2, and 3
	 * 
	 * @param speed
	 *            is a double between -1 and 1 negative is reverse, positive if
	 *            forward, 0 is stationary
	 */
	public void driveLeft(double speed) {
		driveleftMotor1(speed);
		driveleftMotor2(speed);
	}

	/**
	 * Calls right motor 1 and creates a local variable "speed" Refers to boolean in
	 * Robot map and if true, speed = - speed Uses set() command to assign the new
	 * speed to right motor 1
	 * 
	 * @param double
	 *            speed between -1 and 1 negative is reverse, positive if forward, 0
	 *            is stationary
	 */
	private void driverightMotor1(double speed) {
		if (RobotMap.DT_REVERSE_RIGHT1)
			speed = -speed;

		rightMotor1.set(speed);
		rightMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;
	}

	/**
	 * Calls right motor 2 and creates a local variable "speed" Refers to boolean in
	 * Robot map and if true, speed = - speed Uses set() command to assign the new
	 * speed to right motor 2
	 * 
	 * @param double
	 *            speed between -1 and 1 negative is reverse, positive if forward, 0
	 *            is stationary
	 */
	private void driverightMotor2(double speed) {
		if (RobotMap.DT_REVERSE_RIGHT2)
			speed = -speed;

		rightMotor2.set(speed);
		rightMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
	}

	/**
	 * Takes in a double speed and sets it to their right motors 1, 2, and 3
	 * 
	 * @param speed
	 *            is a double between -1 and 1 negative is reverse, positive if
	 *            forward, 0 is stationary
	 */
	public void driveRight(double speed) {
		driverightMotor1(speed);
		driverightMotor2(speed);
	}

	/**
	 * Takes in speed for right and speed for left and sets them to their respective
	 * sides
	 * 
	 * @param leftSpeed
	 *            is a double between -1 and 1
	 * @param rightSpeed
	 *            is a double between -1 and 1 negative is reverse, positive if
	 *            forward, 0 is stationary
	 */
	public void dangerousTankDrive(double leftSpeed, double rightSpeed) {
		driveLeft(leftSpeed);
		driveRight(rightSpeed);
		
	}
	
	public void tankDrive(double leftSpeed, double rightSpeed) {	
		driveLeft(leftSpeed);
		driveRight(rightSpeed);
	}

	/**
	 * Calls for default command of the drivetrain to be DriveWithJoystick
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoystick(0));
	}

	/**
	 * returns total distance traveled by right side of drivetrain
	 * 
	 * @return double in feet of total distance traveled by right encoder
	 */
	public double getRightPosition() {
		return drivetrainRightEncoder.getPos();
	}

	/**
	 * returns total distance traveled by left side of drivetrain
	 * 
	 * @return double in feet of total distance traveled by left encoder
	 */
	public double getLeftPosition() {
		return drivetrainLeftEncoder.getPos();
	}

	/**
	 * returns total distance traveled by drivetrain
	 * 
	 * @return double in inches of average distance traveled by both encoders
	 */
	public double getAverageDistance() {
		return imu.getPos();
	}

	/**
	 * resets position of right encoder to 0 inches
	 */
	public void resetRightPosition() {
		drivetrainRightEncoder.reset();
	}

	/**
	 * resets position of left encoder to 0 inches
	 */
	public void resetLeftPosition() {
		drivetrainLeftEncoder.reset();
	}

	/**
	 * resets position of both Encoders to 0 inches
	 */
	public void resetPosition() {
		resetLeftPosition();
		resetRightPosition();
	}

	/**
	 * Gets the voltage given by the sonar sensor on the Gear Intake.
	 * 
	 * @return the raw voltage from the gear presence sensor
	 */
	public double getSonarVoltage() {
		return DrivetrainSonarSensor.getVoltage();
	}
	
	/**
	 * Gets the status of the line detector 
	 * @return true if line is detected
	 */
	public boolean getLinedectorStatus() {
		if(INVERT_LINE_SENSOR) {
			return !lineDetector.get();
		} else {
			return lineDetector.get();
		}
	}

	/**
	 * returns heading of robot
	 * 
	 * @return double between 0 degrees and 360 degrees
	 */
	public double getHeading() {
		return gyroSPI.getPos();
	}

	/**
	 * Reset robot heading to zero.
	 */
	public void resetGyro() {
		gyroSPI.reset();
	}

	/**
	 * Calibrate gyro. This should only be called if the robot will be stationary
	 * for the calibration period.
	 */
	public void calibrateGyro() {
		gyroSPI.calibrate();
	}

	/**
	 * @return true if the gyro completed its previous calibration sequence.
	 */
	public boolean isGyroCalibrated() {
		return gyroSPI.hasCompletedCalibration();
	}

	/**
	 * @return true if the gyro is being calibrated.
	 */
	public boolean isGyroCalibrating() {
		return gyroSPI.isCalibrating();
	}

	/**
	 * Call to stop an active gyro calibration sequence.
	 */
	public void stopGyroCalibrating() {
		gyroSPI.stopCalibrating();
	}

	/**
	 * Returns the last commanded voltage of left Motor 1
	 * 
	 * @return Double in volts between 0 and 12
	 */
	public double getleftMotor1Voltage() {
		return leftMotor1Voltage;
	}

	/**
	 * Returns the last commanded voltage of left Motor 2
	 * 
	 * @return Double in volts between 0 and 12
	 */
	public double getleftMotor2Voltage() {
		return leftMotor2Voltage;
	}

	/**
	 * Returns the last commanded voltage of right Motor 1
	 * 
	 * @return Double in volts between 0 and 12
	 */
	public double getrightMotor1Voltage() {
		return rightMotor1Voltage;
	}

	/**
	 * Returns the last commanded voltage of right Motor 2
	 * 
	 * @return Double in volts between 0 and 12
	 */
	public double getrightMotor2Voltage() {
		return rightMotor2Voltage;
	}

	public double getRightEncoderRate() {
		return drivetrainRightEncoder.getRate();
	}

	public double getLeftEncoderRate() {
		return drivetrainLeftEncoder.getRate();
	}

	public double getAverageEncoderRate() {
		return ((getRightEncoderRate() + getLeftEncoderRate()) / 2);
	}

	/**
	 * Returns the current position of the gun style controller interpolated
	 * 
	 * @param x
	 *            is voltage
	 * @return Potentiometer position
	 */

	
	
	/**
	 * Call to start an active gyro calibration sequence.
	 */
	public void startGyroCalibrating() {
		gyroSPI.startCalibrating();
	}
	
	
	public double returnRightEncoderRate()
	{
		return getRightEncoderRate();
	}
	
	public double returnLeftEncoderRate() {
	return getLeftEncoderRate();	
	}		
		
	public double PIDVoltagefeedRightMotor1 () {
		if(getRightEncoderRate() != 0)
			return this.getrightMotor1Voltage() / this.getRightEncoderRate();
		else 
			return 0.0;
	}
	
	public double PIDVoltagefeedRightMotor2 () {
		if(getRightEncoderRate() != 0)
			return this.getrightMotor2Voltage() / this.getRightEncoderRate();
		else 
			return 0.0;
	}
	
	public double PIDVoltagefeedLeftMotor1 () {
		if(getLeftEncoderRate() != 0)
			return this.getleftMotor1Voltage() / this.getLeftEncoderRate();
		else 
			return 0.0;
	}
	
	public double PIDVoltagefeedLeftMotor2 () {
		if(getLeftEncoderRate() != 0)
			return this.getleftMotor2Voltage() / this.getLeftEncoderRate();
		else 
			return 0.0;
	}
}




