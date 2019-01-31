/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.robot.subsystems;

import org.team2168.PID.sensors.ADXRS453Gyro;
import org.team2168.PID.sensors.AverageEncoder;
import org.team2168.robot.RobotMap;
import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //Hardware
    private static PWMSpeedController _leftMotor1;
    private static PWMSpeedController _leftMotor2;
    private static PWMSpeedController _leftMotor3;
    private static PWMSpeedController _rightMotor1;
    private static PWMSpeedController _rightMotor2; 
    private static PWMSpeedController _rightMotor3;

    private ADXRS453Gyro _gyroSPI;
    private AverageEncoder _drivetrainLeftEncoder;
    private AverageEncoder _drivetrainRightEncoder;


    public volatile double _leftMotor1Voltage;
    public volatile double _leftMotor2Voltage;
    public volatile double _leftMotor3Voltage;
	  public volatile double _rightMotor1Voltage;
    public volatile double _rightMotor2Voltage;
    public volatile double _rightMotor3Voltage;


    private static Drivetrain _instance = null;

    /**
     * default constructor
     */
    private Drivetrain()
    {
      _leftMotor1 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_1);
      _leftMotor2 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_2);
      _leftMotor3 = new VictorSP(RobotMap.LEFT_DRIVE_MOTOR_3);
      _rightMotor1 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_1);
      _rightMotor2 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_2);
      _rightMotor3 = new VictorSP(RobotMap.RIGHT_DRIVE_MOTOR_3);

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
     * 
     * @return double in feet of total distance traveled by right encoder
     */
    public double getRightPosition()
    {
      return _drivetrainRightEncoder.getPos();
    }
    

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
