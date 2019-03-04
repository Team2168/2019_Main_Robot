
package org.team2168.commands.monkeyBarPivot;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveRotateMonkeyBarWithJoystick extends Command
{
  public DriveRotateMonkeyBarWithJoystick()
  {

    requires(Robot.monkeyBarPivot);
  }

  @Override
  protected void initialize() {
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  @Override
  protected void execute() {
    Robot.monkeyBarPivot.driveRotateBarMotors(Robot.oi.getDriveMonkeyBarPivotJoystickValue());
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
