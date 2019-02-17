
package org.team2168.Commands;

import org.team2168.robot.OI;
import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveCargoPickupWithJoystick extends Command {
  public DriveCargoPickupWithJoystick() {

    requires(Robot.monkeybar);
  }

  @Override
  protected void initialize() {
    Robot.monkeybar.driveIntakeAll(0.0);
  }


  @Override
  protected void execute() {
    Robot.monkeybar.driveIntakeAll(OI.getDriveIntakeWheelsJoystickValue());
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
