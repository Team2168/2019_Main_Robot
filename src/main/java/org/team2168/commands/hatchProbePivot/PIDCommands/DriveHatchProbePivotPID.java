/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchProbePivot.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveHatchProbePivotPID extends Command {

  private double setPoint;

  private double maxSpeed;
  private double minSpeed;
  private double error = 0.5; //Rotational degree error, 0 never ends

  private boolean absolute = true;

  public DriveHatchProbePivotPID() {
 // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hatchProbePivot);
    this.setPoint = Robot.hatchProbePivot.hatchProbePivotController.getSetPoint();
    this.maxSpeed = 1;
    this.minSpeed = 0;
    
  }

  public DriveHatchProbePivotPID(double setPoint)
  {
    this.setPoint = setPoint;
  }

  public DriveHatchProbePivotPID(double setPoint, double maxSpeed)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
  }

  public DriveHatchProbePivotPID(double setPoint, double maxSpeed, double minSpeed)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
  }

  public DriveHatchProbePivotPID(double setPoint, double maxSpeed, double minSpeed, boolean absolute)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
    this.absolute = absolute;
  }

  public DriveHatchProbePivotPID(double setPoint, double maxSpeed, double minSpeed, double error)
  {
    this.setPoint = setPoint;
    this.maxSpeed = maxSpeed;
    this.minSpeed = minSpeed;
    this.error = error;
  }

  public DriveHatchProbePivotPID(double setPoint, double maxSpeed, double minSpeed, double error, boolean absolute)
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
      sp = this.setPoint + Robot.hatchProbePivot.getPotPos();
    else
      sp = this.setPoint;
    
    Robot.hatchProbePivot.hatchProbePivotController.reset();

    Robot.hatchProbePivot.hatchProbePivotController.setpGain(RobotMap.HP_PIVOT_P);
    Robot.hatchProbePivot.hatchProbePivotController.setiGain(RobotMap.HP_PIVOT_I);
    Robot.hatchProbePivot.hatchProbePivotController.setdGain(RobotMap.HP_PIVOT_D);
    Robot.hatchProbePivot.hatchProbePivotController.setSetPoint(sp);
    Robot.hatchProbePivot.hatchProbePivotController.setMaxPosOutput(maxSpeed);
    Robot.hatchProbePivot.hatchProbePivotController.setMaxNegOutput(-maxSpeed);
    Robot.hatchProbePivot.hatchProbePivotController.setMinPosOutput(minSpeed);
    Robot.hatchProbePivot.hatchProbePivotController.setMinNegOutput(-minSpeed);
    Robot.hatchProbePivot.hatchProbePivotController.setAcceptErrorDiff(error);

    Robot.hatchProbePivot.hatchProbePivotController.Enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hatchProbePivot.drivePlungerArmPivotMotor(Robot.hatchProbePivot.hatchProbePivotController.getControlOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.hatchProbePivot.hatchProbePivotController.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.hatchProbePivot.hatchProbePivotController.Pause();
    Robot.hatchProbePivot.drivePlungerArmPivotMotor(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
