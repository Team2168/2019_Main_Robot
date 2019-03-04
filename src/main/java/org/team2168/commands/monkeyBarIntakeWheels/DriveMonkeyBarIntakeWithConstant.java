package org.team2168.commands.monkeyBarIntakeWheels;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarIntakeWithConstant extends Command {
  
  double _speed;
  public DriveMonkeyBarIntakeWithConstant(double inputspeed) {

    requires(Robot.monkeyBarIntakeWheels);
    _speed = inputspeed;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.monkeyBarIntakeWheels.driveIntakeAll(_speed);
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
