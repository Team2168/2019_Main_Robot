
package org.team2168.Commands;

import org.team2168.robot.OI;
import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveRotateMonkeyBarWithJoystick extends Command {
  public DriveRotateMonkeyBarWithJoystick() {

    requires(Robot.monkeybar);
  }

  @Override
  protected void initialize() {
    Robot.monkeybar.driveRotateBarMotors(0.0);
  }

  @Override
  protected void execute() {
    Robot.monkeybar.driveRotateBarMotors(OI.getDriveIntakePivotJoystickValue());
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.monkeybar.driveRotateBarMotors(0.0);
  }


  @Override
  protected void interrupted() {
  }
}
