/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.lift;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.lift.PIDCommands.DriveLiftPIDZZZ;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveLiftToLvl2Position extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveLiftToLvl2Position() {
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
    {
      addSequential(new DriveLiftPIDZZZ(RobotMap.LIFT_LVL_2_POS_PBOT,  RobotMap.LIFT_PID_SPEED_UP_MAX,
        RobotMap.LIFT_PID_SPEED_DOWN_MAX, RobotMap.LIFT_PID_SPEED_UP_MIN, RobotMap.LIFT_PID_SPEED_DOWN_MIN, 
        RobotMap.LIFT_PID_ERROR, true));
    }
    else
    {
      addSequential(new DriveLiftPIDZZZ(RobotMap.LIFT_LVL_2_POS,  RobotMap.LIFT_PID_SPEED_UP_MAX,
      RobotMap.LIFT_PID_SPEED_DOWN_MAX, RobotMap.LIFT_PID_SPEED_UP_MIN, RobotMap.LIFT_PID_SPEED_DOWN_MIN, 
      RobotMap.LIFT_PID_ERROR, true));
    }
  }
}
