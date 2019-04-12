
package org.team2168.commands.monkeyBarIntakeWheels;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveMonkeyBarIntakeWithJoystick extends Command {
  public DriveMonkeyBarIntakeWithJoystick() 
  {
    requires(Robot.monkeyBarIntakeWheels);
  }

  @Override
  protected void initialize() 
  {
    Robot.monkeyBarIntakeWheels.driveIntakeAll(0.0);
  }


  /**
   * Drives intake wheels with opertor manual control
   * 
   * When climb is enabled, the driver gun style trigger will command the motors to allow the driver to climb
   */
  protected void execute() 
  {
    if(Robot.isClimbEnabled)
      if(Math.abs(Robot.oi.driverJoystick.getLeftStickRaw_Y())>0.1) //we only want to drive fwd, never reverse
        Robot.monkeyBarIntakeWheels.driveIntakeAll((Math.abs(Robot.oi.driverJoystick.getLeftStickRaw_Y()+.2)) + 0.15);
      else
        Robot.monkeyBarIntakeWheels.driveIntakeAll(0.07); //always drive wheels in climb mode
    else
      Robot.monkeyBarIntakeWheels.driveIntakeAll(0.6*Robot.oi.getMonkeyBarIntakeJoystickValue());
  }


  @Override
  protected boolean isFinished() 
  {
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
