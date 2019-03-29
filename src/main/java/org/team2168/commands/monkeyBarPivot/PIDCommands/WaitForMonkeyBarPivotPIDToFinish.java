/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.monkeyBarPivot.PIDCommands;

import org.team2168.Robot;
import org.team2168.subsystems.MonkeyBarPivot;

import edu.wpi.first.wpilibj.command.Command;


public class WaitForMonkeyBarPivotPIDToFinish extends Command {
  public WaitForMonkeyBarPivotPIDToFinish() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(MonkeyBarPivot.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.monkeyBarPivot.monkeyBarPivotController.isFinished() || !Robot.monkeyBarPivot.monkeyBarPivotController.isEnabled();
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
