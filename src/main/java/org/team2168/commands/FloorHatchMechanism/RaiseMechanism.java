/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.Commands.FloorHatchMechanism;

import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RaiseMechanism extends Command {
  public RaiseMechanism() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.floorHatchMechanism);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.floorHatchMechanism.raise();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.floorHatchMechanism.isSolenoidRaised();
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