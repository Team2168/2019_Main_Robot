/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Engages stingers and puts drivetrain in neutral
 */
public class DisengageStingers extends Command
{
  public DisengageStingers()
  {
    requires(Robot.shifterStinger);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
    Robot.shifterStinger.disengageStingers();
    if(Robot.habClimbPattern.isRunning())
    {
      Robot.habClimbPattern.cancel();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute()
  {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished()
  {
    return Robot.shifterStinger.isStingerDisengaged();
  }

  // Called once after isFinished returns true
  @Override
  protected void end()
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted()
  {
  }
}
