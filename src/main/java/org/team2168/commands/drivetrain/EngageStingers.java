/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.LEDs.HABClimbPattern;

import edu.wpi.first.wpilibj.command.Command;



/**
 * Engages stingers and puts drivetrain in neutral
 */
public class EngageStingers extends Command
{
  public EngageStingers()
  {
    requires(Robot.shifterStinger);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
    Robot.shifterStinger.engageStingers();
    Robot.habClimbPattern.start();
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
    return Robot.shifterStinger.isStingerEngaged();
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
