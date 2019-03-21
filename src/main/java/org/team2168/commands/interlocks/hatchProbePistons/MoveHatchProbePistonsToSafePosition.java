/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.interlocks.hatchProbePistons;

import org.team2168.Robot;
import org.team2168.commands.hatchProbePistons.RetractHatchPlunger;

import edu.wpi.first.wpilibj.command.CommandGroup;

//retracts hatch probe in order to prevent collision with monkey bar or lift
public class MoveHatchProbePistonsToSafePosition extends CommandGroup {
  public MoveHatchProbePistonsToSafePosition() {
  if(Robot.isPracticeRobot())
    addSequential(new RetractHatchPlunger());
  else
    addSequential(new RetractHatchPlunger());
  }
}
