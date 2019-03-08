
package org.team2168.commands.monkeyBarPivot;

import org.team2168.Robot;
import org.team2168.subsystems.MonkeyBarPivot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarPivotWithJoystick extends Command
{
  public DriveMonkeyBarPivotWithJoystick()
  {

    requires(MonkeyBarPivot.getInstance());
  }

  @Override
  protected void initialize() {
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  @Override
  protected void execute() {
    if(Robot.isClimbEnabled)
      Robot.monkeyBarPivot.driveRotateBarMotors(-Robot.oi.driverJoystick.getX(Hand.kLeft)*0.75);
    else
      Robot.monkeyBarPivot.driveRotateBarMotors(Robot.oi.getMonkeyBarPivotJoystickValue());
  }

  @Override
  protected boolean isFinished()
  {
    return false;
  }

  @Override
  protected void end() {
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  @Override
  protected void interrupted()
  {
  }
}
