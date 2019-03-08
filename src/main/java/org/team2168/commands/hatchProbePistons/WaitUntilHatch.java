/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchProbePistons;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

//this command grabs the hatch 
public class WaitUntilHatch extends Command
{
  public WaitUntilHatch()
  {
    requires(Robot.hatchProbePistons);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute()
  {
    Robot.hatchProbePistons.HatchDisengaged();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished()
  {
    return Robot.hatchProbePistons.isHatchPresent();
  }

  // Called once after isFinished returns true
  @Override
  protected void end()
  {
    Robot.hatchProbePistons.HatchEngaged();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted()
  {

  }
}
