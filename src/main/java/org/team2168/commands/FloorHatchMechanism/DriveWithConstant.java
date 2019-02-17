/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.FloorHatchMechanism;

import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithConstant extends Command {

  private int _speed;
  public DriveWithConstant(int inputSpeed) {
    requires(Robot.floorHatchMechanism);
    _speed = inputSpeed;
  }

 
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    Robot.floorHatchMechanism.intakeHatchPanel(_speed);
  }


  @Override
  protected boolean isFinished() {
      return false;
    }


  @Override
  protected void end() {
    Robot.floorHatchMechanism.intakeHatchPanel(0);
  }


  @Override
  protected void interrupted() {
    end();
  }
}
