/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.commands.monkeyBarPivot.DriveMonkeyBarPivotWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
* Add your docs here.
*/
public class MonkeyBarPivot extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSP _rotateBarLeft;
  private VictorSP _rotateBarRight;

  private AveragePotentiometer _monkeyBarRotationRight;


  public volatile double _rotateBarLeftVoltage;
  public volatile double _rotateBarRightVoltage;

  private static MonkeyBarPivot _instance;

  //constructors for monkey bar
  private MonkeyBarPivot()
  {
    _rotateBarLeft = new VictorSP(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP);
    _rotateBarRight = new VictorSP(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP);
    
    if(Robot.isPracticeRobot())
    {

      _monkeyBarRotationRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_0_PBOT,
        RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0_PBOT,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX_PBOT,
        RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT,
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

    }
    else
    {

      _monkeyBarRotationRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_0,
        RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0,
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX,
        RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION,
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

    }
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

    _rotateBarLeft.set(speed);
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

    _rotateBarRight.set(speed);
    _rotateBarRightVoltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  /**
   * Drives both monkey bar intake wheels, where 1 is out and -1 is in
   */
  public void driveRotateBarMotors(double speed)
  {
    driveRotateMotorLeft(speed);
    driveRotateMotorRight(speed);
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
}
