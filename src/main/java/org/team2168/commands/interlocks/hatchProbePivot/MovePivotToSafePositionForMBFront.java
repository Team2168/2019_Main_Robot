/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.interlocks.hatchProbePivot;

import org.team2168.RobotMap;
import org.team2168.Robot;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

//prevents pivot colliding with monkey bar as monkey is stowed and pivot is on mb side
public class MovePivotToSafePositionForMBFront extends CommandGroup {
  public MovePivotToSafePositionForMBFront() {
  if(Robot.isPracticeRobot())
    addSequential(new DriveMonkeyBarPivotPID(RobotMap.MONKEY_BAR_SAFE_SCORING_POS_PBOT, 0.5, 0, true));
  else
    addSequential(new DriveMonkeyBarPivotPID(RobotMap.MONKEY_BAR_SAFE_SCORING_POS, 0.5, 0, true));

  }
}
