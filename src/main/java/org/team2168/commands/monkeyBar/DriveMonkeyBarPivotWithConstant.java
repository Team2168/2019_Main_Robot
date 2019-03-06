
package org.team2168.commands.monkeyBar;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarPivotWithConstant extends Command {
  double speed;
  public DriveMonkeyBarPivotWithConstant(double inputspeed) {

    requires(Robot.monkeybar);
    speed = inputspeed;

  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.monkeybar.driveRotateBarMotors(speed);
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
