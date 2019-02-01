/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.robot.Robot;
import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoysticks extends Command {

  int ctrlStyle;
    /**
     * Controller Styles: 0 = TankDrive (Default), 1 = Gunstyle, 2 = Arcade drive,
     * 3 = GTA, 4 = New Gun Style
     * 
     * @param inputStyle
     */
  private double angle;
  private double endDistance;
  private double distanceGoal;
  private double speed;
  private double error = 0.1;

  double lastRotateOutput;

  public DriveWithJoysticks(int inputStyle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    ctrlStyle = inputStyle;
    this.distanceGoal = 1;
    this.speed = RobotMap.AUTO_NORMAL_SPEED;
    this.lastRotateOutput = 0;
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    ctrlStyle = Robot.getControlStyleInt();
    switch(ctrlStyle)
    {
      /**
       * Initialize driveStraightController for Gun Style
       */
    case 1:
            Robot.drivetrain.getInstance();

            //reset controller 
                Robot.drivetrain.resetPosition();
                Robot.drivetrain.imu.reset();
                Robot.drivetrain.driveTrainPosController.reset();
                Robot.drivetrain.rotateDriveStraightController.reset();
            
            endDistance = Robot.drivetrain.getAverageDistance() + distanceGoal;
            angle = Robot.drivetrain.getHeading();

            Robot.drivetrain.driveTrainPosController.setSetPoint(endDistance);
            Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
            Robot.drivetrain.driveTrainPosController.setMaxNegOutput(speed);
            Robot.drivetrain.driveTrainPosController.setAcceptErrorDiff(error);
            Robot.drivetrain.rotateDriveStraightController.setSetPoint(angle);

            Robot.drivetrain.driveTrainPosController.Enable();

    default: 

        break;
    

    }

  }

  // Called repeatedly when this Command is scheduled to run
  /**
   * ////////////QUESTION!!
   * Why do we sometimes use tankDrive (which limits speed when 
   * the lift is up as a protective measure) and sometimes use driveRight and driveLeft,
   * separately, which don't have this protection?
   * 
   * IE: in last year's code styles Tank Drive, arcade drive,
   *  and GTA drive use driveRight and driveLeft separately
   * 
   * Just Gun Style and New Gun Style use tankDrive
   * 
   * Update: Aidan told me to go ahead and use tankDrive for all of them. Please correct if 
   * someone else disagrees
   * ////////////////////
   */
  @Override
  protected void execute() {
    ctrlStyle = Robot.getControlStyleInt();
    switch (ctrlStyle)
    {
      /**
       * Tank Drive
       */
      case 0:

              Robot.drivetrain.tankDrive(Robot.oi.driverJoystick.getLeftStickRaw_Y(), 
                  Robot.oi.driverJoystick.getRightStickRaw_Y());
              break;

      /**
       * Gun Style
       * 
       * X values: /////FROM LAST YEAR
       * full in: -0.516
       * nothing: 0.354 & 0.342
       * full out: 0.622
       */
      case 1:

              lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
              if ((Robot.oi.driverJoystick.getLeftStickRaw_Y() < 0.1) && (Robot.oi.driverJoystick.getLeftStickRaw_Y() > -0.1))
              {
                Robot.drivetrain.tankDrive(Robot.oi.getGunStyleXValue(), Robot.oi.getGunStyleXValue());
              }
              else 
              {
                Robot.drivetrain.tankDrive(
                                (Robot.oi.getGunStyleXValue() + Robot.oi.driverJoystick.getLeftStickRaw_Y()),
                                (Robot.oi.getGunStyleXValue() - Robot.oi.driverJoystick.getLeftStickRaw_Y()));
                Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
              }
      
      /**
       * Arcade Drive
       * 
       */
      case 2: 
              Robot.drivetrain.tankDrive(
                      (Robot.oi.driverJoystick.getLeftStickRaw_Y() + Robot.oi.driverJoystick.getRightStickRaw_X()),
                      (Robot.oi.driverJoystick.getLeftStickRaw_Y() - Robot.oi.driverJoystick.getRightStickRaw_X()));
              break;

      /**
       * GTA Drive
       */
      case 3:
              double fwdSpeed = Robot.oi.driverJoystick.getRightTriggerAxisRaw();
              double revSpeed = Robot.oi.driverJoystick.getLeftTriggerAxisRaw();
              double speed = fwdSpeed - revSpeed;
              double rotation = Robot.oi.driverJoystick.getRightStickRaw_X();

              //Adjusts angle while moving
              if (speed != 0 && rotation != 0)
              {
                Robot.drivetrain.tankDrive((rotation * speed), (-rotation * speed));
              }

              //Allows robot to spin in place without needing to press triggers
              else if (speed == 0 && rotation != 0)
              {
                Robot.drivetrain.tankDrive(rotation, -rotation);
              }

              //Allows robot to drive straight
              else if (speed != 0 && rotation == 0)
              {
                Robot.drivetrain.tankDrive(speed, speed);
              }

              break;
      /**
       * New Gun Style
       */
      case 4:
              lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
            
              if (Math.abs(Robot.oi.driverJoystick.getX(Hand.kLeft)) < 0.1)
              {
                Robot.drivetrain.tankDrive(-Robot.oi.driverJoystick.getY(Hand.kLeft),
                               -Robot.oi.driverJoystick.getY(Hand.kLeft));
              }
              else 
              {
                //Arcade drive
                Robot.drivetrain.tankDrive(
                                Robot.oi.getGunStyleYValue() + Robot.oi.driverJoystick.getX(Hand.kLeft),
                                Robot.oi.getGunStyleYValue() - Robot.oi.driverJoystick.getX(Hand.kLeft));
                Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
                
              }

              break;
      
      /**
       * Defaults to tank drive
       */
      default: 
                Robot.drivetrain.tankDrive(Robot.oi.driverJoystick.getLeftStickRaw_Y(), 
                                Robot.oi.driverJoystick.getRightStickRaw_Y());
                break;

              




    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
