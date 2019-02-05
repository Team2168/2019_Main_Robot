/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands;


import org.team2168.robot.Robot;
import org.team2168.subsystems.lift;

import edu.wpi.first.wpilibj.command.Command;

public class liftConstant extends Command {
  double speed;
  public liftConstant(double _liftSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.Lift);
    speed=_liftSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    lift.driveLift(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    lift.driveLift(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    lift.driveLift(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    lift.driveLift(0.0);
  }
}
