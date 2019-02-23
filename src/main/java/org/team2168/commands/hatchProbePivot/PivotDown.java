/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchProbePivot;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class PivotDown extends Command {
  private double _speed;
  public PivotDown(double inputSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    /**
     * make the sign of inputSpeed whatever you want the default side to be 
     * ie, if/when it is perfectly in the middle, if you would want it to go front,
     * inputSpeed should be positive, and vice versa
     */
    requires(Robot.hatchProbePivot);
    _speed = inputSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //on front side
    if(Robot.hatchProbePivot.getPotPos() < 90 && _speed < 0)
    {
      _speed = -_speed;
    }
    else if(Robot.hatchProbePivot.getPotPos() > 90 && _speed > 0)
    {
      _speed = -_speed;
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.hatchProbePivot.getPotPos() < 90 && Robot.hatchProbePivot.isPivotHallEffectForward())
      return true;
    else if(Robot.hatchProbePivot.getPotPos() > 90 && Robot.hatchProbePivot.isPivotHallEffectReverse())
      return true;
    else 
      return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.hatchProbePivot.drivePlungerArmPivotMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
