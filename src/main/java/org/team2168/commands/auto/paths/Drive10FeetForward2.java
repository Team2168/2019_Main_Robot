/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.auto.paths;

import org.team2168.Robot;
import org.team2168.commands.drivetrain.PIDCommands.DrivePIDPathQuinticPID;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Drive10FeetForward2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Drive10FeetForward2() {
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
    addSequential(new DrivePIDPathQuinticPID(Robot.leftPosQuinticPath3, Robot.rightPosQuinticPath3, Robot.leftVelQuinticPath3,
          Robot.rightVelQuinticPath3, Robot.leftAccQuinticPath3, Robot.rightAccQuinticPath3, Robot.headingQuinticPath3, Robot.reversePath3, Robot.limelightEnable3));
  }
}
