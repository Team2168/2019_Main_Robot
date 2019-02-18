/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.Lift;

import org.team2168.robot.OI;
import org.team2168.robot.Robot;
import org.team2168.robot.RobotMap;
import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

public class LiftJoystick extends Command {
  public LiftJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.lift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Lift.driveLift(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Lift.driveLift(OI.getDriveLiftJoystickValue()*RobotMap.LIFT_MAX_JOYSTICK_SPEED);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Lift.driveLift(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
