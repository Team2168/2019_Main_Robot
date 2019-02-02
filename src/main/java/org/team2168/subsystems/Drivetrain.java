/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.sensors.ADXRS453Gyro;
import org.team2168.PID.sensors.AverageEncoder;
import org.team2168.PID.sensors.IMU;
import org.team2168.commands.drivetrain.DriveWithJoysticks;
import org.team2168.robot.RobotMap;
import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //Hardware
    private static SpeedController _leftMotor1;
    private static SpeedController _leftMotor2;
    private static SpeedController _leftMotor3;
    private static SpeedController _rightMotor1;
    private static SpeedController _rightMotor2; 
    private static SpeedController _rightMotor3;

    private ADXRS453Gyro _gyroSPI;
    private AverageEncoder _drivetrainLeftEncoder;
    private AverageEncoder _drivetrainRightEncoder;

  //voltage
    public volatile double _leftMotor1Voltage;
    public volatile double _leftMotor2Voltage;
    public volatile double _leftMotor3Voltage;
	  public volatile double _rightMotor1Voltage;
    public volatile double _rightMotor2Voltage;
    public volatile double _rightMotor3Voltage;

    public IMU _imu;

    // declare position/speed controllers
	public PIDPosition _drivetrainPosController;
//	public PIDPosition _rotateController;
	public PIDPosition _rotateDriveStraightController;


    private static Drivetrain _instance = null;

    /**
     * default constructor
     */
    private Drivetrain()
    {
      if (Robot.isPracticeRobot())
      {
        _leftMotor1 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_1);
        _leftMotor2 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_2);
        _leftMotor3 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_3);
        _rightMotor1 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_1);
        _rightMotor2 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_2);
        _rightMotor3 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_3);

      }
      else if (Robot.isCanDrivetrain())
      {
        _leftMotor1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_1_CAN);
        _leftMotor2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_2_CAN);
        _leftMotor3 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_3_CAN);
        _rightMotor1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_1_CAN);
        _rightMotor2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_2_CAN);
        _rightMotor3 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_3_CAN);
      }
      else 
      {
        _leftMotor1 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_1);
        _leftMotor2 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_2);
        _leftMotor3 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_3);
        _rightMotor1 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_1);
        _rightMotor2 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_2);
        _rightMotor3 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_3);
      }
      

      _gyroSPI = new ADXRS453Gyro();
      _gyroSPI.startThread();

      _drivetrainRightEncoder = new AverageEncoder(
				RobotMap.RIGHT_DRIVE_ENCODER_A,
				RobotMap.RIGHT_DRIVE_ENCODER_B,
				RobotMap.DRIVE_ENCODER_PULSE_PER_ROT,
				RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
				RobotMap.RIGHT_DRIVE_TRAIN_ENCODER_REVERSE,
				RobotMap.DRIVE_ENCODING_TYPE,
				RobotMap.DRIVE_SPEED_RETURN_TYPE,
				RobotMap.DRIVE_POS_RETURN_TYPE,
				RobotMap.DRIVE_AVG_ENCODER_VAL);


		  _drivetrainLeftEncoder = new AverageEncoder(
				RobotMap.LEFT_DRIVE_ENCODER_A, 
				RobotMap.LEFT_DRIVE_ENCODER_B,
				RobotMap.DRIVE_ENCODER_PULSE_PER_ROT, 
				RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
				RobotMap.LEFT_DRIVE_TRAIN_ENCODER_REVERSE, 
				RobotMap.DRIVE_ENCODING_TYPE,
				RobotMap.DRIVE_SPEED_RETURN_TYPE, 
				RobotMap.DRIVE_POS_RETURN_TYPE, 
        RobotMap.DRIVE_AVG_ENCODER_VAL);
        
        _imu = new IMU(_drivetrainLeftEncoder, _drivetrainRightEncoder, RobotMap.WHEEL_BASE);

        // rotateController = new PIDPosition(
				// "RotationController", 
        // RobotMap.ROTATE_POSITION_P, 
				// RobotMap.ROTATE_POSITION_I,
				// RobotMap.ROTATE_POSITION_D, 
				// _gyroSPI, 
				// RobotMap.DRIVE_TRAIN_PID_PERIOD);

		
		_rotateDriveStraightController = new PIDPosition(
				"RotationStraightController",
				RobotMap.ROTATE_POSITION_P_Drive_Straight, 
				RobotMap.ROTATE_POSITION_I_Drive_Straight,
				RobotMap.ROTATE_POSITION_D_Drive_Straight, 
				_gyroSPI, 
				RobotMap.DRIVE_TRAIN_PID_PERIOD);

		_drivetrainPosController = new PIDPosition(
				"drivetrainPosController", 
				RobotMap.DRIVE_TRAIN_RIGHT_POSITION_P,
				RobotMap.DRIVE_TRAIN_RIGHT_POSITION_I, 
				RobotMap.DRIVE_TRAIN_RIGHT_POSITION_D, 
				_imu,
        RobotMap.DRIVE_TRAIN_PID_PERIOD);
        

    // add min and max output defaults and set array size
		
		_drivetrainPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
		//rotateController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
		_rotateDriveStraightController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);


		// start controller threads
		
		_drivetrainPosController.startThread();
	//	rotateController.startThread();
		_rotateDriveStraightController.startThread();


    }

    /**
	  * Calls instance object and makes it a singleton object of type Drivetrain
	  * 
	  * @returns Drivetrain object "instance"
	  */
	  public static Drivetrain getInstance()
	  {
		  if (_instance == null)
			  _instance = new Drivetrain();
		  return _instance;
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
    public void driveLeftMotor1(double speed)
    {
      if (RobotMap.DT_REVERSE_LEFT1)
        speed = -speed;
      
      _leftMotor1.set(speed);
      _leftMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;

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
    public void driveLeftMotor2(double speed)
    {
      if (RobotMap.DT_REVERSE_LEFT2)
        speed = -speed;
      _leftMotor2.set(speed);
      _leftMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
    }

    /**
	  * Calls left motor 3 and creates a local variable "speed" Refers to boolean in
	  * Robot map and if true, speed = - speed Uses set() command to assign the new
	  * speed to left motor 3
	  * 
	  * @param double
	  *            speed between -1 and 1 negative is reverse, positive if forward, 0
	  *            is stationary
	  */
    public void driveLeftMotor3(double speed)
    {
      if (RobotMap.DT_REVERSE_LEFT3)
        speed = -speed;
      _leftMotor3.set(speed);
      _leftMotor3Voltage = Robot.pdp.getBatteryVoltage() * speed;
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
    public void driveRightMotor1(double speed)
    {
      if (RobotMap.DT_REVERSE_RIGHT1)
        speed = -speed;
      _rightMotor1.set(speed);
      _rightMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;
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
    public void driveRightMotor2(double speed)
    {
      if (RobotMap.DT_REVERSE_RIGHT2)
        speed = -speed;
      _rightMotor2.set(speed);
      _rightMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
    }

    /**
	  * Calls right motor 3 and creates a local variable "speed" Refers to boolean in
	  * Robot map and if true, speed = - speed Uses set() command to assign the new
	  * speed to right motor 3
	  * 
	  * @param double
	  *            speed between -1 and 1 negative is reverse, positive if forward, 0
	  *            is stationary
	  */
    public void driveRightMotor3(double speed)
    {
      if (RobotMap.DT_REVERSE_RIGHT3)
        speed = -speed;
      _rightMotor3.set(speed);
      _rightMotor3Voltage = Robot.pdp.getBatteryVoltage() * speed;
    }

    /**
     * This method is to drive the left motor controllers together
     * @param speed is a value between 1 and -1 as a double, 
     * where 1 represents forward motion and -1 represents backward motion
     */
    public void driveLeftMotors(double speed)
    {
      driveLeftMotor1(speed);
      driveLeftMotor2(speed);
      driveLeftMotor3(speed);
    }

    /**
     * This method is to drive the right motor controllers together
     * @param speed is a value between 1 and -1 as a double, 
     * where 1 represents forward motion and -1 represents backward motion
     */
    public void driveRightMotors(double speed)
    {
      driveRightMotor1(speed);
      driveRightMotor2(speed);
      driveRightMotor3(speed);
    }

    /**
	  * This method is to drive the two sides, left and right, of the drivetrain simultaneously at two different speeds. 
	  * @param leftSpeed is a value between -1 and 1 as a double, where 1 indicates forward motion.
	  * @param rightSpeed is a value between -1 and 1 as a double, where 1 indicates forward motion.
	  */
    public void dangerousTankDrive(double leftSpeed, double rightSpeed)
    {
      driveLeftMotors(leftSpeed);
      driveRightMotors(rightSpeed);

    }

    /////////////Check for this years lift height//////////////////////////////////////////
    /**
  	 * Tank drive that limits speed when lift is up.
  	 * @param leftSpeed
  	 * @param rightSpeed
  	 */
    public void tankDrive(double leftSpeed, double rightSpeed)
    {
      if (Robot.lift.getPotPos() > 30)     //Won't work until lift is integrated
      {
        leftSpeed = leftSpeed * 0.3;
        rightSpeed = rightSpeed * 0.3;
      }

      driveLeftMotors(leftSpeed);
      driveRightMotors(rightSpeed);
    }

    
    /**
     * returns heading of robot
     * 
     * @return double between 0 and 360 degrees
     */
    public double getHeading()
    {
      return _gyroSPI.getPos();
    }

    /**
     * Reset robot heading to zero
     * 
     */
    public void resetGyro()
    {
      _gyroSPI.reset();
    }

    /**
     * Calibrate Gyro. This should only be called if the robot will be stationary for the 
     * calibration period. 
     */
    public void calibrateGyro()
    {
      _gyroSPI.calibrate();
    }

    /**
     * call to start an active calibration sequence
     */
    public void startGyroCalibrating()
    {
      _gyroSPI.startCalibrating();;
    }

    /**
     * 
     * @return true if the gyro has completed its previous calibration sequence
     */
    public boolean isGyroCalibrated()
    {
      return _gyroSPI.hasCompletedCalibration();
    }

    /**
     * 
     * @return true if the gyro is being calibrated
     */
    public boolean isGyroCalibrating()
    {
      return _gyroSPI.isCalibrating();
    }

    /**
     * Call to stop an active gyro calibration sequence
     */
    public void stopGyroCalibrating()
    {
      _gyroSPI.stopCalibrating();
    }

    /**
     * returns total distance traveled by the right side of the drivetrain
     * 
     * @return double in feet of total distance traveled by right encoder
     */
    public double getRightPosition()
    {
      return _drivetrainRightEncoder.getPos();
    }

    /**
     * returns total distance traveled by the right side of the drivetrain
     * 
     * @return double in feet of total distance traveled by right encoder 
     */
    public double getLeftPosition()
    {
      return _drivetrainLeftEncoder.getPos();
    }

    /**
     * returns total distance traveled by drivetrain
     * 
     * @return double in inches of average distance traveled by both encoders
     */
    public double getAverageDistance()
    {
      return _imu.getPos();
    }
    
    /**
     * resets position of right encoder to 0 inches
     */
    public void resetRightPosition()
    {
      _drivetrainRightEncoder.reset();
    }

    /**
     * resets position of left encoder of 0 inches
     */
    public void resetLeftPosition()
    {
      _drivetrainLeftEncoder.reset();
    }

    /**
     * resets position of both encoders to 0 inches
     */
    public void resetPosition()
    {
      resetLeftPosition();
      resetRightPosition();
    }

    // Code from last year that I don't know if we will use, and I can't find the associated classes
  //   /**
	//  * Gets the voltage given by the sonar sensor on the Gear Intake.
	//  * 
	//  * @return the raw voltage from the gear presence sensor
	//  */
	// public double getSonarVoltage() {
	// 	return DrivetrainSonarSensor.getVoltage();
	// }
	
	// /**
	//  * Gets the status of the line detector 
	//  * @return true if line is detected
	//  */
	// public boolean getLinedectorStatus() {
	// 	if(INVERT_LINE_SENSOR) {
	// 		return !lineDetector.get();
	// 	} else {
	// 		return lineDetector.get();
	// 	}
  // }
  
  /**
   * returns the last commanded voltage of Left motor 1
   * 
   * @return double in volts between 0 and 12
   */
  public double getLeftMotor1Voltage()
  {
    return _leftMotor1Voltage;
  }

  /**
   * returns the last commanded voltage of left motor 2
   * 
   * @return double in volts between 0 and 12
   */
  public double getLeftMotor2Voltage()
  {
    return _leftMotor2Voltage;
  }

  /**
   * returns the last commanded voltage of left motor 3
   * 
   * @return double in volts between 0 and 12
   */
  public double getLeftMotor3Voltage()
  {
    return _leftMotor3Voltage;
  }

  /**
   * returns the last commanded voltage of right motor 1
   * 
   * @return double in volts between 0 and 12
   */
  public double getRightMotor1Voltage()
  {
    return _rightMotor1Voltage;
  }

  /**
   * returns the last commanded voltage of right motor 2
   * 
   * @return double in volts between 0 and 12
   */
  public double getRightMotor2Voltage()
  {
    return _rightMotor2Voltage;
  }

  /**
   * returns the last commanded voltage of right motor 3
   * 
   * @return double in volts between 0 and 12
   */
  public double getRightMotor3Voltage()
  {
    return _rightMotor3Voltage;
  }

  public double getRightEncoderRate()
  {
    return _drivetrainRightEncoder.getRate();
  }

  public double getLeftEncoderRate()
  {
    return _drivetrainLeftEncoder.getRate();
  }

  public double getAverageEncoderRate()
  {
    return ((getRightEncoderRate()+ getLeftEncoderRate()) /2);
  }

  public double PIDVoltageFeedLeftMotor1()
  {
    if (getLeftEncoderRate() != 0)
    {
      return getLeftMotor1Voltage() / getLeftEncoderRate();
    }
    else 
      return 0.0;
  }

  public double PIDVoltageFeedLeftMotor2()
  {
    if (getLeftEncoderRate() != 0)
    {
      return getLeftMotor2Voltage() / getLeftEncoderRate();
    }
    else
      return 0.0;
  }

  public double PIDVoltageFeedLeftMotor3()
  {
    if (getLeftEncoderRate() != 0)
    {
      return getLeftMotor3Voltage() / getLeftEncoderRate();
    }
    else
      return 0.0;
  }

  public double PIDVoltageFeedRightMotor1()
  {
    if (getRightEncoderRate() != 0)
    {
      return getRightMotor1Voltage() / getRightEncoderRate();
    }
    else
      return 0.0;
  }

  public double PIDVoltageFeedRightMotor2()
  {
    if (getRightEncoderRate() != 0)
    {
      return getRightMotor2Voltage() / getRightEncoderRate();
    }
    else 
      return 0.0;
  }

  public double PIDVoltageFeedRightMotor3()
  {
    if (getRightEncoderRate() != 0)
    {
      return getRightMotor3Voltage() / getRightEncoderRate();
    }
    else
      return 0.0;
  }
  






  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoysticks(0));
  }
}
