
package org.team2168.commands.monkeyBarPivot;

import org.team2168.Robot;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.MonkeyBarPivot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarPivotWithConstant extends Command {
  double _speed;

  public DriveMonkeyBarPivotWithConstant(double inputspeed) {

    requires(MonkeyBarPivot.getInstance());
    _speed = inputspeed;

  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.monkeyBarPivot.driveRotateBarMotors(_speed);
    //System.out.println("Driving MonkeyBar with Constant");
  }

  @Override
  protected boolean isFinished() {
    if (_speed == 0.0)
      return true;
    else
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
