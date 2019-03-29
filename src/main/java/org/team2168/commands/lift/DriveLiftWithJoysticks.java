/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.lift;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

public class DriveLiftWithJoysticks extends Command
{
  public DriveLiftWithJoysticks()
  {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Lift.getInstance());
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
    if(RobotMap.LIFT_ENABLE_HEIGHT_HOLD)
    {
      double holdingSpeed = RobotMap.LIFT_HOLDING_VOLTAGE/Robot.pdp.getBatteryVoltage();
      if(Math.abs(Math.abs(Robot.oi.getLiftJoystickValue()))<holdingSpeed)
      {
        if(Lift.getInstance().getPotPos() <= RobotMap.LIFT_ZERO_BELOW_THIS_HEIGHT)
        {
          Robot.lift.driveAllMotors(-holdingSpeed);//+0.01
        //  System.out.println("Da lift is holding down");
        }
        else
        {
          Robot.lift.driveAllMotors(holdingSpeed);
         // System.out.println("Da lift is holding up");
        }
       
      }
      else
        Robot.lift.driveAllMotors(Robot.oi.getLiftJoystickValue() * RobotMap.LIFT_MAX_JOYSTICK_SPEED);
    }
    else
      Robot.lift.driveAllMotors(Robot.oi.getLiftJoystickValue() * RobotMap.LIFT_MAX_JOYSTICK_SPEED);
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
    end();
  }
}
