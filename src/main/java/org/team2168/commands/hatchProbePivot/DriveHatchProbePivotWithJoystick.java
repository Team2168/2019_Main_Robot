/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchProbePivot;

import org.team2168.OI; 
import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveHatchProbePivotWithJoystick extends Command {
  public DriveHatchProbePivotWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hatchProbePivot);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {

    if(RobotMap.PLUNGER_ARM_PIVOT_ENABlE_HEIGHT_HOLD)
    {
      double holdingSpeed = RobotMap.PLUNGER_ARM_PIVOT_HOLDING_VOLTAGE/Robot.pdp.getBatteryVoltage();
      if(Math.abs(Robot.oi.getHatchProbePivotJoystickValue())<holdingSpeed)
        Robot.hatchProbePivot.drivePlungerArmPivotMotor(Math.cos(Math.toRadians(Robot.hatchProbePivot.getPotPos()))*holdingSpeed);
      else
        Robot.hatchProbePivot.drivePlungerArmPivotMotor(Robot.oi.getHatchProbePivotJoystickValue());
    }
    else
      Robot.hatchProbePivot.drivePlungerArmPivotMotor(Robot.oi.getHatchProbePivotJoystickValue());
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.hatchProbePivot.drivePlungerArmPivotMotor(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
