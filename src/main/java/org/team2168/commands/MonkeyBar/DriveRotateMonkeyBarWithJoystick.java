
package org.team2168.commands.monkeyBar;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveRotateMonkeyBarWithJoystick extends Command
{
  public DriveRotateMonkeyBarWithJoystick()
  {

    requires(Robot.monkeybar);
  }

  @Override
  protected void initialize()
  {
    Robot.monkeybar.driveRotateBarMotors(0.0);
  }

  @Override
  protected void execute()
  {
    Robot.monkeybar.driveRotateBarMotors(Robot.oi.getDriveIntakePivotJoystickValue());
  }

  @Override
  protected boolean isFinished()
  {
    return false;
  }

  @Override
  protected void end()
  {
    Robot.monkeybar.driveRotateBarMotors(0.0);
  }

  @Override
  protected void interrupted()
  {
  }
}
