package org.team2168.commands.monkeyBar;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarWheelsWithConstant extends Command
{

  double _speed;

  public DriveMonkeyBarWheelsWithConstant(double inputspeed)
  {

    requires(Robot.monkeybar);
    _speed = inputspeed;
  }

  @Override
  protected void initialize()
  {
  }

  @Override
  protected void execute()
  {
    Robot.monkeybar.driveIntakeAll(_speed);
  }

  @Override
  protected boolean isFinished()
  {
    return false;
  }

  @Override
  protected void end()
  {
    Robot.monkeybar.driveIntakeAll(0.0);
  }

  @Override
  protected void interrupted()
  {
  }
}
