/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.LEDs;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class PivotingPattern extends Command {
  public PivotingPattern() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.leds);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //trigger when the speed of the pivot is a certain threshhold away from zero pos or neg
    Robot.leds.writePattern(RobotMap.PATTERN_CONFETTI_RAINBOW);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
