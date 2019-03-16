
package org.team2168.commands.monkeyBarPivot;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.subsystems.MonkeyBarPivot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarPivotWithJoystick extends Command
{

  public static final double TRIGGER_OFFSET = 2.65;

  public DriveMonkeyBarPivotWithJoystick()
  {

    requires(MonkeyBarPivot.getInstance());
  }

  @Override
  protected void initialize() {
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  @Override
  protected void execute() {
    if(Robot.isClimbEnabled && RobotMap.ONE_TRIGGER_CLIMB_ENABLED)
    { 
      double triggerValue;
      if(Robot.oi.getGunStyleYValue()>0.05)
        triggerValue = Robot.oi.getGunStyleYValue();
      else
        triggerValue = 0;
      //Allow trigger to drive monkey bar, and mix with steering wheel for fine adjustment
      Robot.monkeyBarPivot.driveRotateBarMotors(-triggerValue*TRIGGER_OFFSET - Robot.oi.driverJoystick.getX(Hand.kLeft)*0.75);
    }
    else if(Robot.isClimbEnabled)
      Robot.monkeyBarPivot.driveRotateBarMotors(-Robot.oi.driverJoystick.getX(Hand.kLeft)*0.75);
    else
      Robot.monkeyBarPivot.driveRotateBarMotors(Robot.oi.getMonkeyBarPivotJoystickValue());
  }

  @Override
  protected boolean isFinished()
  {
    return false;
  }

  @Override
  protected void end() {
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  @Override
  protected void interrupted()
  {
  }
}
