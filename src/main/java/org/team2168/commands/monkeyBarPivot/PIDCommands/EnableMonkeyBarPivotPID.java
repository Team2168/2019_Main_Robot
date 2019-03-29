/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.monkeyBarPivot.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.subsystems.MonkeyBarPivot;

import edu.wpi.first.wpilibj.command.Command;

public class EnableMonkeyBarPivotPID extends Command {

  private double setPoint;

  private double maxSpeed;
  private double minSpeed;
  private double error = 0.5; //Rotational degree error, 0 never ends

  private boolean absolute = true;

  public EnableMonkeyBarPivotPID() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(MonkeyBarPivot.getInstance());
    this.setPoint = Robot.monkeyBarPivot.monkeyBarPivotController.getSetPoint();
    this.maxSpeed = 0.5;
    this.minSpeed = 0;
    
  }

  public EnableMonkeyBarPivotPID(double setPoint)
  {
    this.setPoint = setPoint;
  }

  public EnableMonkeyBarPivotPID(double setPoint, double maxSpeed)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
  }

  public EnableMonkeyBarPivotPID(double setPoint, double maxSpeed, double minSpeed)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
  }

  public EnableMonkeyBarPivotPID(double setPoint, double maxSpeed, double minSpeed, boolean absolute)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
    this.absolute = absolute;
  }

  public EnableMonkeyBarPivotPID(double setPoint, double maxSpeed, double minSpeed, double error)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
    this.error = error;
  }

  public EnableMonkeyBarPivotPID(double setPoint, double maxSpeed, double minSpeed, double error, boolean absolute)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
    this.error = error;
    this.absolute = absolute;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double sp = 0;
    if (!absolute)
      sp = this.setPoint + Robot.monkeyBarPivot.getRightPotPos();
    else
      sp = this.setPoint;
    
    Robot.monkeyBarPivot.monkeyBarPivotController.reset();

    Robot.monkeyBarPivot.monkeyBarPivotController.setpGain(RobotMap.MB_PIVOT_P);
    Robot.monkeyBarPivot.monkeyBarPivotController.setiGain(RobotMap.MB_PIVOT_I);
    Robot.monkeyBarPivot.monkeyBarPivotController.setdGain(RobotMap.MB_PIVOT_D);
    Robot.monkeyBarPivot.monkeyBarPivotController.setSetPoint(sp);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMaxPosOutput(maxSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMaxNegOutput(-maxSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMinPosOutput(minSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMinNegOutput(-minSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setAcceptErrorDiff(error);

    Robot.monkeyBarPivot.monkeyBarPivotController.Enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.monkeyBarPivot.driveRotateBarMotors(Robot.monkeyBarPivot.monkeyBarPivotController.getControlOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.monkeyBarPivot.monkeyBarPivotController.Pause();
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
