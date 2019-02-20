/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchProbePivot;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class TogglePivotDirection extends Command {
  private int _counter = 0;
  private double _speed;
  private boolean _allowToggleFront;
  private boolean _allowToggleBack;
  private boolean _findSpeed;
  private boolean _reverseSpeed;
  public TogglePivotDirection(double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hatchProbePivot);
    _speed = speed;
    _allowToggleFront = false;
    _allowToggleBack = false;
    _findSpeed = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (_speed == 0)
      _counter++;
    else if(_speed != 0)
      _counter = 0;
    if (_counter > RobotMap.PIVOT_STOP_RESET_TIME)
    {
      if(Robot.hatchProbePivot.isPivotHallEffectForward())
      {
        _allowToggleFront = true;
        _allowToggleBack = false;
        _findSpeed = true;
      }
      else if(Robot.hatchProbePivot.isPivotHallEffectReverse())
      {
        _allowToggleBack = true;
        _allowToggleFront = false;
        _findSpeed = true;
      }
        
    }
    if(_findSpeed && _allowToggleFront)
    {
      if(_speed >0)
      {
        _reverseSpeed = true;
        _findSpeed = false;
      }
      else if(_speed < 0)
      {
        _reverseSpeed = false;
        _findSpeed = false;
      }
    }
    if(_findSpeed && _allowToggleBack)
    {
      if(_speed < 0)
      {
        _reverseSpeed = true;
        _findSpeed = false;
      }
      else if(_speed > 0 )
      {
        _reverseSpeed = false;
        _findSpeed = false;
      }
    }
    if(_reverseSpeed)
      _speed = -_speed;
    Robot.hatchProbePivot.drivePlungerArmPivotMotor(_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
