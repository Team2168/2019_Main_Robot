/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RotatePIDLimelight extends Command {

  private double setPoint;
  private double minSpeed;
  private double maxSpeed;
  private double leftSpeed;
  private double rightSpeed;
  private double error = 0.5; // Rotational degree error, default 0 never ends
  
  /**
   * Default constructor
   */
  public RotatePIDLimelight() {
    requires(Robot.drivetrain);

    this.setPoint = 0;
    this.minSpeed = -0.5;
    this.maxSpeed = 0.5;
    this.leftSpeed = 0;
    this.rightSpeed = 0;
  }

  /**
   * Constructor
   * @param leftSpeed is the left motor speed at which the drivetrain would drive straight
   * @param rightSpeed is the right motor speed at which the drivetrain would drive straight
   */
  public RotatePIDLimelight(double leftSpeed, double rightSpeed) {
    requires(Robot.drivetrain);

    this.setPoint = 0;
    this.minSpeed = -0.5;
    this.maxSpeed = 0.5;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  /**
   * Constructor
   * @param minSpeed is the minimum speed the drivetrain can drive at
   * @param maxSpeed is the maximum speed the drivetrain can drive at
   * @param leftSpeed is the left motor speed at which the drivetrain would drive straight
   * @param rightSpeed is the right motor speed at which the drivetrain would drive straight
   */
  public RotatePIDLimelight(double minSpeed, double maxSpeed, double leftSpeed, double rightSpeed) {
    requires(Robot.drivetrain);

    this.setPoint = 0;
    this.minSpeed = minSpeed;
    this.maxSpeed = maxSpeed;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.limelightPosController.reset();

    Robot.drivetrain.limelightPosController.setSetPoint(setPoint);
		Robot.drivetrain.limelightPosController.setMaxPosOutput(maxSpeed);
		Robot.drivetrain.limelightPosController.setMaxNegOutput(-maxSpeed);
		Robot.drivetrain.limelightPosController.setMinPosOutput(minSpeed);
		Robot.drivetrain.limelightPosController.setMinNegOutput(-minSpeed);
    Robot.drivetrain.limelightPosController.setAcceptErrorDiff(error);
    
    Robot.drivetrain.limelightPosController.Enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Rotates the drivetrain at the speed outputted by the position controller in order to face the targets
    Robot.drivetrain.tankDrive(-Robot.drivetrain.limelightPosController.getControlOutput(),
       Robot.drivetrain.limelightPosController.getControlOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drivetrain.limelightPosController.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.limelightPosController.Pause();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}
