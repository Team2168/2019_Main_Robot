/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.StingerRatchet;

import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DisengageStingerRatchet extends Command {
  public DisengageStingerRatchet() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.stingerRatchet);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.stingerRatchet.disengageRatchet();;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.stingerRatchet.isRatchetDisengaged();
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
