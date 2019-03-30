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
    //if speed is negative (we are trying to drive down), so we only allow the monkey bar
    //to drive down if we are above the zero height
    //else if we are positive we are trying to drive up, we only allow the pivot to drive up
    //if we are less than the max height
  if((speed<-0.1 && getRightPotPos() > RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0) ||
    (speed>0.1 && getRightPotPos() < RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION))
    {
      driveRotateMotorLeft(speed); 
      driveRotateMotorRight(speed);
    }
    else
    {
      driveRotateMotorLeft(0.0);
      driveRotateMotorRight(0.0);
    }
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
}
