/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.monkeyBarPivot.interlocks;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPID;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPIDPath;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveMonkeyBarToCargoIntakePosition extends CommandGroup {

  public MoveMonkeyBarToCargoIntakePosition()
  {
    if(Robot.isPracticeRobot())
      addSequential(new DriveMonkeyBarPivotPID(RobotMap.MONKEY_BAR_CARGO_INTAKE_POS_PBOT, 0.4, 0, true));
    else
      //addSequential(new DriveMonkeyBarPivotPID(RobotMap.MONKEY_BAR_CARGO_INTAKE_POS, 0.4, 0, true));
      addSequential(new DriveMonkeyBarPivotPIDPath(40));
  }


  
  
}
