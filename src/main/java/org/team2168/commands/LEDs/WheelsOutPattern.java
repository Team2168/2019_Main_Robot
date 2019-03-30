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

public class WheelsOutPattern extends Command {
  public WheelsOutPattern() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.leds);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //trigger when the floor hatch intake wheels go out a certain threshhold from zero
    if(RobotMap.LEDS_REVERSE)
    {
      Robot.leds.writePatternOneColor(RobotMap.PATTERN_ANIMATED_WAVE, 96, 255, 255);
    }
    else
      Robot.leds.writePatternOneColor(RobotMap.PATTERN_ANIMATED_WAVE_REVERSE, 96, 255, 255);
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
