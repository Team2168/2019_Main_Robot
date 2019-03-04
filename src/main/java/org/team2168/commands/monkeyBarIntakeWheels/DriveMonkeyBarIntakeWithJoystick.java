
package org.team2168.commands.monkeyBarIntakeWheels;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarIntakeWithJoystick extends Command {
  public DriveMonkeyBarIntakeWithJoystick() {

    requires(Robot.monkeyBarIntakeWheels);
  }

  @Override
  protected void initialize() {
    Robot.monkeyBarIntakeWheels.driveIntakeAll(0.0);
  }


  @Override
  protected void execute() {
    Robot.monkeyBarIntakeWheels.driveIntakeAll(Robot.oi.getDriveIntakeWheelsJoystickValue());
  }


  @Override
  protected boolean isFinished() {
    return false;
  }


  @Override
  protected void end() {
    Robot.monkeyBarIntakeWheels.driveIntakeAll(0.0);
  }


  @Override
  protected void interrupted() {
  }
}
