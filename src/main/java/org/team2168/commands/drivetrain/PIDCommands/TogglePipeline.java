/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TogglePipeline extends Command {

  private boolean finished;

  /**
   * Default constructor
   */
  public TogglePipeline() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    finished = false;

    // If vision is active, switches to a raw pipeline
    if(Robot.drivetrain.limelight.getPipeline() == 0) {
      Robot.drivetrain.limelight.setPipeline(7);
      finished = true;
    }
    // If vision is inactive, switches to the vision pipeline
    else if(Robot.drivetrain.limelight.getPipeline() == 7) {
      Robot.drivetrain.limelight.setPipeline(0);
      finished = true;
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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
