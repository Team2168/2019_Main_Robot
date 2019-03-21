/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.interlocks.hatchProbePivot;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.hatchProbePivot.PIDCommands.DriveHatchProbePivotPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

//prevents pivot colliding with monkey bar as monkey is stowed and pivot is on back side
public class MovePivotToSafePositionForMBBack extends CommandGroup {
  public MovePivotToSafePositionForMBBack() {
  if(Robot.isPracticeRobot())
    addSequential(new DriveHatchProbePivotPID(RobotMap.PLUNGER_ARM_SAFE_POS_MB_BACK_PBOT, 0.5, 0, true));
  else
    addSequential(new DriveHatchProbePivotPID(RobotMap.PLUNGER_ARM_SAFE_POS_MB_BACK, 0.5, 0, true));

  }
}
