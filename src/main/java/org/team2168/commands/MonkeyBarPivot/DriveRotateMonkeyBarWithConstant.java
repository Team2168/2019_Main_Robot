
package org.team2168.commands.MonkeyBarPivot;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveRotateMonkeyBarWithConstant extends Command {
  double _speed;
  public DriveRotateMonkeyBarWithConstant(double inputspeed) {

    requires(Robot.monkeyBarPivot);
    _speed = inputspeed;

  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.monkeyBarPivot.driveRotateBarMotors(_speed);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }


  @Override
  protected void interrupted() {
  }
}
