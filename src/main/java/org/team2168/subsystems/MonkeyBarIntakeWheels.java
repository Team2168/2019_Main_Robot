
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
* Add your docs here.
*/
public class MonkeyBarIntakeWheels extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX _intakeLeft;
  private VictorSPX _intakeRight;

  public volatile double _intakeLeftVoltage;
  public volatile double _intakeRightVoltage;

  private static MonkeyBarIntakeWheels _instance;

  //constructors for monkey bar
  private MonkeyBarIntakeWheels()
  {
    _intakeLeft = new VictorSPX(RobotMap.MONKEY_BAR_INTAKE_WHEELS_LEFT_PDP);
    _intakeRight = new VictorSPX(RobotMap.MONKEY_BAR_INTAKE_WHEELS_RIGHT_PDP);



    

    //ConsolePrinter.putNumber("Monkey Bar Intake Joystick value", () -> {return Robot.oi.getMonkeyBarIntakeJoystickValue();}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Intake right motor voltage", () -> {return _intakeRightVoltage;}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Intake left motor voltage", () -> {return _intakeLeftVoltage;}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Intake right motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_INTAKE_WHEELS_RIGHT_PDP);}, true, false);
    ConsolePrinter.putNumber("Monkey Bar Intake left motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_INTAKE_WHEELS_LEFT_PDP);}, true, false);
  }

  /**
    * Singleton constructor of the plunger arm pivot
    *
  */
  public static MonkeyBarIntakeWheels getInstance() {
    if (_instance == null)
      _instance = new MonkeyBarIntakeWheels();
    return _instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveMonkeyBarIntakeWithJoystick());
  }

  /**
   * Takes in a double speed and sets it to the left monkey bar intake wheels
   * @param speed is a double between -1 and 1, where positive is forward motion
   */
  public void driveIntakeMotorLeft(double speed)
  {
    if(RobotMap.MONKEY_BAR_INTAKE_LEFT_REVERSE)
      speed = -speed;

    _intakeLeft.set(ControlMode.PercentOutput,speed);
    _intakeLeftVoltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  /**
   * Takes in a double speed and sets it to the right monkey bar intake wheels
   * @param speed is a double between -1 and 1, where positive is forward motion
   */
  public void driveIntakeMotorRight(double speed)
  {
    if(RobotMap.MONKEY_BAR_INTAKE_RIGHT_REVERSE)
      speed = -speed;

    _intakeRight.set(ControlMode.PercentOutput,speed);
    _intakeRightVoltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  /**
   * Drives both monkey bar intake wheels, where 1 is out and -1 is in
   */
  public void driveIntakeAll(double speed)
  {
    driveIntakeMotorLeft(speed);
    driveIntakeMotorRight(speed);
  }
}

