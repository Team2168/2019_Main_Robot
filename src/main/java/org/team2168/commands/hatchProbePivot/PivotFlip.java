/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchProbePivot;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class PivotFlip extends Command {
  private boolean _goFront;
  private double _speed;
  public PivotFlip(double inputSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hatchProbePivot);
    //make _goFront equal whatever you want the default to be should the pivot be in the middle
    _goFront = true;
    _speed = inputSpeed;  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.hatchProbePivot.getPotPos() > 90)
      _goFront = true;
    else if(Robot.hatchProbePivot.getPotPos() < 90)
      _goFront = false;
    
    if(_goFront)
      _speed = Math.abs(_speed);
    else 
      _speed = -Math.abs(_speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hatchProbePivot. drivePlungerArmPivotMotor(_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(_goFront && Robot.hatchProbePivot.isPivotHallEffectForward())
      return true;
    else if(!_goFront && Robot.hatchProbePivot.isPivotHallEffectReverse())
      return true;
    else
      return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
