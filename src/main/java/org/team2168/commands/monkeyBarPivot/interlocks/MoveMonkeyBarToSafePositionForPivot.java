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

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveMonkeyBarToSafePositionForPivot extends CommandGroup {
  public MoveMonkeyBarToSafePositionForPivot() {
  if(Robot.isPracticeRobot())
    addSequential(new DriveMonkeyBarPivotPID(RobotMap.MONKEY_BAR_SAFE_PIVOT_POS_PBOT, 1, 0, true));
  else
    addSequential(new DriveMonkeyBarPivotPID(RobotMap.MONKEY_BAR_SAFE_PIVOT_POS, 1, 0, true));

  }
    
}
