/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.lift;

import org.team2168.Robot;
import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

/**
 * DriveLiftWithConstant is a command which drives the lift to a certain height
 * when a button is pressed (e.g. pressing a moves the lift to the height of the
 * first level of the rocket) the code here is a placeholder in the case where
 * we want a command to drive the lift with a constant
 */
public class DriveLiftWithConstant extends Command
{
  double _speed;

  public DriveLiftWithConstant(double speed)
  {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Lift.getInstance());

    _speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
    Robot.lift.driveAllMotors(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute()
  {
    // this is just a placeholder
    Robot.lift.driveAllMotors(_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished()
  {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end()
  {
    Robot.lift.driveAllMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted()
  {
    Robot.lift.driveAllMotors(0.0);
  }
}
