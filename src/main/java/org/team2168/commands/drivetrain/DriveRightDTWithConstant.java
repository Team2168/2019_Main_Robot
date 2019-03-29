/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveRightDTWithConstant extends Command
{

  private double _speed;

  public DriveRightDTWithConstant(double inputSpeed, boolean isVoltage)
  {
    requires(Robot.drivetrain);
    if(isVoltage)
      _speed = inputSpeed/Robot.pdp.getBatteryVoltage();
    else
      _speed = inputSpeed;
  }

  @Override
  protected void initialize()
  {

  }

  @Override
  protected void execute()
  {
    Robot.drivetrain.driveRight(_speed);
  }

  @Override
  protected boolean isFinished()
  {
    return false;
  }

  @Override
  protected void end()
  {
    Robot.drivetrain.driveRight(0);
  }

  @Override
  protected void interrupted()
  {
    end();
  }
}
