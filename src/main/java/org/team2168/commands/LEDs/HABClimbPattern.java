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

public class HABClimbPattern extends Command {
  public HABClimbPattern() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.leds);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //add logic to trigger when stingers engage??--somepoint in the HAB climb--
    if (Robot.onBlueAlliance())
    {
      if(RobotMap.LEDS_REVERSE)
        Robot.leds.writePatternOneColor(RobotMap.PATTERN_ROCKET_ASCEND, 160, 255, 200);
      else  
        Robot.leds.writePatternOneColor(RobotMap.PATTERN_ROCKET_DESCEND, 160, 255, 200);
    }
    else
    {
      if(RobotMap.LEDS_REVERSE)
        Robot.leds.writePatternOneColor(RobotMap.PATTERN_ROCKET_ASCEND, 0, 255, 200);
      else  
        Robot.leds.writePatternOneColor(RobotMap.PATTERN_ROCKET_DESCEND, 0, 255, 200);
    }
 
    
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
