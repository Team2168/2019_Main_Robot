/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class EnableLimelight extends Command {

  private double setPoint;
  private double minSpeed;
  private double maxSpeed;
  private double error = 0.5; // Rotational degree error, default 0 never ends

  /**
   * Default constructor
   */
  public EnableLimelight() {

    this.setPoint = 0;
    this.minSpeed = -0.5;
    this.maxSpeed = 0.5;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Limelight is enabled");

    Robot.drivetrain.limelightPosController.reset();

    Robot.drivetrain.limelightPosController.setSetPoint(setPoint);
    Robot.drivetrain.limelightPosController.setMaxPosOutput(maxSpeed);
    Robot.drivetrain.limelightPosController.setMaxNegOutput(-maxSpeed);
    Robot.drivetrain.limelightPosController.setMinPosOutput(minSpeed);
    Robot.drivetrain.limelightPosController.setMinNegOutput(-minSpeed);
    Robot.drivetrain.limelightPosController.setAcceptErrorDiff(error);

    Robot.drivetrain.limelight.setCamMode(0);
    Robot.drivetrain.limelight.setLedMode(0);
    // if(Robot.driverstation.isFMSAttached())
    // {
      if(Robot.onBlueAlliance())
      {
        Robot.drivetrain.limelight.setPipeline(3);
      }
      else
      {
        Robot.drivetrain.limelight.setPipeline(2);
      }
  //   }
  //   else
  //   {
  //     Robot.drivetrain.limelight.setPipeline(0);
  //   }
    Robot.drivetrain.limelightPosController.Enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drivetrain.limelightPosController.isEnabled();
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
