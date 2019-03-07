/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.monkeyBarPivot.interlocks;

import org.team2168.subsystems.MonkeyBarPivot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveMonkeyBarToSafePositionForLift extends Command {

  double counter = 0;
  public MoveMonkeyBarToSafePositionForLift() {
    // Use requires() here to declare subsystem dependencies
    requires(MonkeyBarPivot.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    counter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (counter < 250)
    {
      counter++;
      System.out.println("Moving Monkey Bar Pivot");
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    
    return counter >= 250;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("*******************************Finished Moving Monkey Bar Pivot\n\n\n\n\n");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
