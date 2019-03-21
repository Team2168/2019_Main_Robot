/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto;

import org.team2168.Robot;
import org.team2168.commands.hatchProbePistons.RetractHatchPlunger;
import org.team2168.commands.hatchProbePivot.MoveHatchProbePivotTo180Position;
import org.team2168.commands.lift.MoveLiftToLvl2Position;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class RotateAndMoveLiftLevel2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RotateAndMoveLiftLevel2() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    if(Robot.hatchProbePistons.isArmExtended())
    {
      addSequential(new RetractHatchPlunger());
    }
    addSequential(new MoveHatchProbePivotTo180Position());
    addSequential(new MoveLiftToLvl2Position());
  }
}
