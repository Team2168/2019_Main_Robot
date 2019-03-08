/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.lift;

import org.team2168.RobotMap;
import org.team2168.Robot;
import org.team2168.commands.lift.PIDCommands.DriveLiftPIDZZZ;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveLiftToCargoShipPosition extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveLiftToCargoShipPosition() {
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
    if(Robot.isPracticeRobot())
      addSequential(new DriveLiftPIDZZZ(RobotMap.LIFT_CARGO_SHIP_POS_PBOT, 1.0, 0.1, 1.0, true));
    else
      addSequential(new DriveLiftPIDZZZ(RobotMap.LIFT_CARGO_SHIP_POS, 1.0, 0.1, 1.0, true));
  }
}
