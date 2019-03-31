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
 * Engages drivetrain and puts stingers in neutral
 */
public class EngageDrivetrain extends Command
{
  public EngageDrivetrain()
  {
    requires(Robot.shifterDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
    Robot.shifterDrivetrain.engageDrivetrain();
    Robot.habClimbPattern.cancel();
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
    return Robot.shifterDrivetrain.isDrivetrainEngaged();
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
