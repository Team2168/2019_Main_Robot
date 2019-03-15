/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.commands.monkeyBarPivot.DriveMonkeyBarPivotWithJoystick;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
* Add your docs here.
*/
public class MonkeyBarPivot extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX _rotateBarLeft;
  private TalonSRX _rotateBarRight;

  private AveragePotentiometer _monkeyBarRotationRight;

  public volatile double _rotateBarLeftVoltage;
  public volatile double _rotateBarRightVoltage;

  private double _safeLiftPosition;
  private double _safePivotPosition;
  private double _safeScoringPosition;
  private double _floorPosition;
  private double _cargoIntakePosition;
  private double _stowPosition;

  private double _errorMargin = 3; //currently 3 degrees to either side of set positions returns true

  public PIDPosition monkeyBarPivotController;
  TCPSocketSender TCPMonkeyBarPivotController;

  private static MonkeyBarPivot _instance;

  private boolean rotateBarLeftFault = false;
  private boolean rotateBarRightFault = false;


  private boolean rotateBarLeftHighCurrent = false;
  private boolean rotateBarRightHighCurrent = false;

  private boolean rotateBarLeftHighThenZeroCurrent = false;
  private boolean rotateBarRightHighThenZeroCurrent = false;


  private boolean isrotateBarLeftBreakerTrip = false;
  private boolean isrotateBarRightBreakerTrip = false;

  //constructors for monkey bar
  private MonkeyBarPivot()
  {
    _rotateBarLeft = new TalonSRX(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP);
    _rotateBarRight = new TalonSRX(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP);
    
    if(Robot.isPracticeRobot())
    {

      _monkeyBarRotationRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_0_PBOT,
        RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0_PBOT,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX_PBOT,
        RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT,
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);
      
      _safeLiftPosition = RobotMap.MONKEY_BAR_SAFE_LIFT_POS_PBOT;
      _safePivotPosition = RobotMap.MONKEY_BAR_SAFE_PIVOT_POS_PBOT;
      _safeScoringPosition = RobotMap.MONKEY_BAR_SAFE_SCORING_POS_PBOT;
      _floorPosition = RobotMap.MONKEY_BAR_FLOOR_POS_PBOT;
      _stowPosition = RobotMap.MONKEY_BAR_STOW_POS_PBOT;
      _cargoIntakePosition = RobotMap.MONKEY_BAR_CARGO_INTAKE_POS_PBOT;

    }
    else
    {

      _monkeyBarRotationRight = new AveragePotentiometer(_rotateBarRight,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_0,
        RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX,
        RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION,
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

      _safeLiftPosition = RobotMap.MONKEY_BAR_SAFE_LIFT_POS;
      _safePivotPosition = RobotMap.MONKEY_BAR_SAFE_PIVOT_POS;
      _safeScoringPosition = RobotMap.MONKEY_BAR_SAFE_SCORING_POS;
      _floorPosition = RobotMap.MONKEY_BAR_FLOOR_POS;
      _stowPosition = RobotMap.MONKEY_BAR_STOW_POS;
      _cargoIntakePosition = RobotMap.MONKEY_BAR_CARGO_INTAKE_POS;

    }

    monkeyBarPivotController = new PIDPosition("MonkeyController", 
      RobotMap.MB_PIVOT_P, 
      RobotMap.MB_PIVOT_I, 
      RobotMap.MB_PIVOT_D,
      _monkeyBarRotationRight, 
      RobotMap.LIFT_PID_PERIOD);

    monkeyBarPivotController.setSIZE(RobotMap.LIFT_PID_ARRAY_SIZE);

    monkeyBarPivotController.startThread();

    TCPMonkeyBarPivotController = new TCPSocketSender(RobotMap.TCP_SERVER_MB_POT_CONTROLLER, monkeyBarPivotController);
    TCPMonkeyBarPivotController.start();

    ConsolePrinter.putNumber("Monkey Bar Pivot Joystick value", () -> {return Robot.oi.getMonkeyBarPivotJoystickValue();}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Pivot right motor voltage", () -> {return _rotateBarRightVoltage;}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Pivot left motor voltage", () -> {return _rotateBarLeftVoltage;}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Pivot right motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP);}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Pivot left motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP);}, true, false);

    ConsolePrinter.putNumber("Monkey Bar Right Raw Pot", () -> { return getRightPotPosRaw();}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Right Pot Degrees", () -> {return getRightPotPos();}, true, false);
    
    ConsolePrinter.putBoolean("Monkey Bar Lowered", () -> {return isLowered();}, true, false);
    ConsolePrinter.putBoolean("Monkey Bar Stowed", () -> {return isStowed();}, true, false);

    ConsolePrinter.putBoolean("MB RotateLeft_FAULT", () -> {return rotateBarLeftFault;}, true, true);
    ConsolePrinter.putBoolean("MB RotateRight_FAULT", () -> {return rotateBarRightFault;}, true, true);


    ConsolePrinter.putBoolean("MB RotateLeft_Breaker_Trip", () -> {return isrotateBarLeftBreakerTrip;}, true, true);
    ConsolePrinter.putBoolean("MB RotateRight_Breaker_Trip", () -> {return isrotateBarRightBreakerTrip;}, true, true);
  }

    /**
   * Singleton constructor of the monkey bar pivot
   *
   */
  public static MonkeyBarPivot getInstance() {
    if (_instance == null)
      _instance = new MonkeyBarPivot();
    return _instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveMonkeyBarPivotWithJoystick());
  }

  /**
   * Takes in a double speed and sets it to the left monkey bar pivot motor
   * @param speed is a double between -1 and 1, where positive is forward motion
   */
  public void driveRotateMotorLeft(double speed)
  {
    if(RobotMap.MONKEY_BAR_ROTATE_LEFT_REVERSE)
      speed = -speed;

    _rotateBarLeft.set(ControlMode.PercentOutput,speed);
    _rotateBarLeftVoltage = Robot.pdp.getBatteryVoltage() * speed;

  }

   /**
   * Takes in a double speed and sets it to the right monkey bar pivot motor
   * @param speed is a double between -1 and 1, where positive is forward motion
   */
  public void driveRotateMotorRight(double speed)
  {
    if(RobotMap.MONKEY_BAR_ROTATE_RIGHT_REVERSE)
    speed = -speed;

    _rotateBarRight.set(ControlMode.PercentOutput,speed);
    _rotateBarRightVoltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  /**
   * Drives both monkey bar intake wheels, where 1 is out and -1 is in
   */
  public void driveRotateBarMotors(double speed)
  {
    if(getRightPotPos()>= RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0)
    {
      driveRotateMotorLeft(speed);
     driveRotateMotorRight(speed);
    }
    else
    {
      driveRotateMotorLeft(0.0);
      driveRotateMotorRight(0.0);
    }

    isRotateBarLeftBreakerTrip();
    isRotateBarRightBreakerTrip();

    isRotateBarLeftFailure();
    isRotateBarRightFailure();
  }

  public boolean isLowered()
  {
    if(Robot.isPracticeRobot())
      return _monkeyBarRotationRight.getPos() == RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0_PBOT;
    else
      return _monkeyBarRotationRight.getPos() == RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0;
    

  }

  public boolean isStowed()
  {
    if(Robot.isPracticeRobot())
      return _monkeyBarRotationRight.getPos() >= RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT;
    else
      return _monkeyBarRotationRight.getPos() >= RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION;
  }

  public double getRightPotPosRaw()
  {
    return _monkeyBarRotationRight.getRawPos();
  }

  public double getRightPotPos()
  {
    return _monkeyBarRotationRight.getPos();
  }

  public boolean isSafeLiftPosition()
  {
    return (getRightPotPos() < _safeLiftPosition);
  }

  public boolean isSafePivotPosition()
  {
    return (getRightPotPos() < _safePivotPosition);
  }

  public boolean isSafeScoringPosition()
  {
    return Math.abs(getRightPotPos() - _safeScoringPosition) < _errorMargin;
  }

  public boolean isFloorPosition()
  {
    return Math.abs(getRightPotPos() - _floorPosition) < _errorMargin;
  }

  public boolean isStowPosition()
  {
    return Math.abs(getRightPotPos() - _stowPosition) < _errorMargin;
  }

  public boolean isCargoIntakePosition()
  {
    return Math.abs(getRightPotPos() - _cargoIntakePosition) < _errorMargin;
  }

  /**
   * The purpose of this method is to compare the current of this motor to that of
   * the other motors in the same gearbox, if it is less than some percentage of
   * the others, it is not driving the same and we throw a fault to be checked
   * later;
   * 
   * Once the fault is thrown, it is not reset until the bot is reset.
   * 
   * TODO: Write to a file for between bot shutdown persistance;
   * 
   * @return
   */
  private void isRotateBarLeftFailure()
  {
    // create a comparison
    double conditionLimitPercent = 0.5;
    if(Robot.isPracticeRobot())
    {
      if (!this.rotateBarLeftFault && this._rotateBarLeftVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED_PBOT)
      {
        this.rotateBarLeftFault = ((Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) <= conditionLimitPercent
            * Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP)
            && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) > 2));
      }
    }
    else
    {
      if (!this.rotateBarLeftFault && this._rotateBarLeftVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED)
      {
        this.rotateBarLeftFault = ((Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) <= conditionLimitPercent
            * Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP)
            && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) > 2));
      }
    }
    
  }

  /**
   * The purpose of this method is to compare the current of this motor to that of
   * the other motors in the same gearbox, if it is less than some percentage of
   * the others, it is not driving the same and we throw a fault to be checked
   * later;
   * 
   * Once the fault is thrown, it is not reset until the bot is reset.
   * 
   * TODO: Write to a file for between bot shutdown persistance;
   * 
   * @return
   */
  private void isRotateBarRightFailure()
  {
    // create a comparison
    double conditionLimtPercent = 0.5;
    if(Robot.isPracticeRobot())
    {
      if (!this.rotateBarRightFault && this._rotateBarRightVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED_PBOT)
      {
        this.rotateBarRightFault = ((Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) <= conditionLimtPercent
            * Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP)
            && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) > 2));
      }
    }
    else
    {
      if (!this.rotateBarRightFault && this._rotateBarRightVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED)
      {
        this.rotateBarRightFault = ((Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) <= conditionLimtPercent
            * Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP)
            && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) > 2));
      }
    }
    
  }

  /**
   * The purpose of this method is to compare the try and determine if we had a
   * tripped breaker which for our purposes has a signature that while driving the
   * motor, we see current, then zero current, then sometime later current.
   * 
   * If we never see current again, we don't assume it is a tripped breaker but
   * rather a blown motor captured by the other motor fault
   * 
   * This is a special case of the motor fault.
   * 
   * TODO: Write to a file for between bot shutdown persistance;
   * 
   * @return
   */
  private void isRotateBarLeftBreakerTrip()
  {
    // we are trying to drive motor
    if (this._rotateBarLeftVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED_PBOT && Robot.isPracticeRobot())
    {
      // did motor ever get to a high current?
      if (Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) > 15)
        rotateBarLeftHighCurrent = true;

      if (rotateBarLeftHighCurrent && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) < 1)
        rotateBarLeftHighThenZeroCurrent = true;

      if (!this.isrotateBarLeftBreakerTrip && rotateBarLeftHighThenZeroCurrent)
        this.isrotateBarLeftBreakerTrip = Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) > 3;

    }
    else if(this._rotateBarLeftVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED && !Robot.isPracticeRobot())
    {
      // did motor ever get to a high current?
      if (Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) > 15)
        rotateBarLeftHighCurrent = true;

      if (rotateBarLeftHighCurrent && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) < 1)
        rotateBarLeftHighThenZeroCurrent = true;

      if (!this.isrotateBarLeftBreakerTrip && rotateBarLeftHighThenZeroCurrent)
        this.isrotateBarLeftBreakerTrip = Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP) > 3;
    }
  }

  /**
   * The purpose of this method is to compare the try and determine if we had a
   * tripped breaker which for our purposes has a signature that while driving the
   * motor, we see current, then zero current, then sometime later current.
   * 
   * If we never see current again, we don't assume it is a tripped breaker but
   * rather a blown motor captured by the other motor fault
   * 
   * This is a special case of the motor fault.
   * 
   * TODO: Write to a file for between bot shutdown persistance;
   * 
   * @return
   */
  private void isRotateBarRightBreakerTrip()
  {
    // we are trying to drive motor
    if (this._rotateBarRightVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED_PBOT && Robot.isPracticeRobot())
    {
      // did motor ever get to a high current?
      if (Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) > 35)
        rotateBarRightHighCurrent = true;

      if (rotateBarRightHighCurrent && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) < 1)
        rotateBarRightHighThenZeroCurrent = true;

      if (!this.isrotateBarRightBreakerTrip && rotateBarRightHighThenZeroCurrent)
        this.isrotateBarRightBreakerTrip = Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) > 3;

    }
    else if(this._rotateBarRightVoltage >= RobotMap.MONKEY_BAR_PIVOT_MIN_SPEED && !Robot.isPracticeRobot())
    {
        // did motor ever get to a high current?
        if (Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) > 35)
          rotateBarRightHighCurrent = true;

        if (rotateBarRightHighCurrent && Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) < 1)
          rotateBarRightHighThenZeroCurrent = true;

        if (!this.isrotateBarRightBreakerTrip && rotateBarRightHighThenZeroCurrent)
          this.isrotateBarRightBreakerTrip = Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP) > 3;
    }
  }

}
