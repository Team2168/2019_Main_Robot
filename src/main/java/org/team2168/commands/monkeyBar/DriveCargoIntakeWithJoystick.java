
package org.team2168.commands.monkeyBar;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveCargoIntakeWithJoystick extends Command {
  public DriveCargoIntakeWithJoystick() {

    requires(Robot.monkeybar);
  }

  @Override
  protected void initialize() {
    Robot.monkeybar.driveIntakeAll(0.0);
  }


  @Override
  protected void execute() {
    Robot.monkeybar.driveIntakeAll(Robot.oi.getDriveIntakeWheelsJoystickValue());
  }


  @Override
  protected boolean isFinished() {
    return false;
  }


  @Override
  protected void end() {
    Robot.monkeybar.driveIntakeAll(0.0);
  }


  @Override
  protected void interrupted() {
  }
}
