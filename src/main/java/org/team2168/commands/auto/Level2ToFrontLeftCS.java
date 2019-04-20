/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto;

import org.team2168.commands.auto.paths.Drive10FeetForward2;
import org.team2168.commands.drivetrain.PIDCommands.DriveDistanceWithLimelight;
import org.team2168.commands.hatchProbePistons.ExtendHatchPlunger;
import org.team2168.commands.hatchProbePistons.ReleaseHatchPanel;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Level2ToFrontLeftCS extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Level2ToFrontLeftCS() {
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
    addSequential(new Drive10FeetForward2());
    addSequential(new DriveDistanceWithLimelight(-12.0, 36.0));
    addSequential(new ExtendHatchPlunger()); //make into one command
    addSequential(new Sleep(), 0.4);
    addSequential(new ReleaseHatchPanel());
  }
}
