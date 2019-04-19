/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.climb;

import org.team2168.commands.auto.Sleep;
import org.team2168.commands.drivetrain.PIDCommands.DriveStingerPIDPath;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithConstant;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPIDPath;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPIDPathAutoClimb;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoClimb() {
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
    addSequential(new DriveMonkeyBarPivotPIDPath(63));
    addSequential(new Sleep(), 0.2);
    addParallel(new DriveStingerPIDPath(7.5,5));
    addSequential(new DriveMonkeyBarPivotPIDPathAutoClimb(63, 0, 5));
    // addParallel(new DriveMonkeyBarIntakeWithConstant(0.3));
    // addSequential(new Sleep(), 1.2);
    //addSequential(new DriveMonkeyBarIntakeWithConstant(0.0));

  }
}
