/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchFloorIntake;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveHatchFloorIntakeWithConstant extends Command
{

  private double _speed;

  public DriveHatchFloorIntakeWithConstant(double inputSpeed)
  {
    requires(Robot.hatchFloorIntake);
    _speed = inputSpeed;
  }

  @Override
  protected void initialize()
  {

  }

  @Override
  protected void execute()
  {
    Robot.hatchFloorIntake.driveMotors(_speed);
  }

  @Override
  protected boolean isFinished()
  {
    return false;
  }

  @Override
  protected void end()
  {
    Robot.hatchFloorIntake.driveMotors(0);
  }

  @Override
  protected void interrupted()
  {
    end();
  }
}
