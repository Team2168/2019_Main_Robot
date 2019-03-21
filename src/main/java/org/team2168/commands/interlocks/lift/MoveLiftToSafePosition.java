/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.interlocks.lift;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.lift.PIDCommands.DriveLiftPIDZZZ;

import edu.wpi.first.wpilibj.command.CommandGroup;

//prevents pivot and monkey bar colliding with lift by lowering lift
public class MoveLiftToSafePosition extends CommandGroup {
  public MoveLiftToSafePosition() {
    if(Robot.isPracticeRobot())
    {
      addSequential(new DriveLiftPIDZZZ(RobotMap.LIFT_LVL_1_POS_PBOT,  RobotMap.LIFT_PID_SPEED_UP_MAX,
        RobotMap.LIFT_PID_SPEED_DOWN_MAX, RobotMap.LIFT_PID_SPEED_UP_MIN, RobotMap.LIFT_PID_SPEED_DOWN_MIN, 
        RobotMap.LIFT_PID_ERROR, true));
    }
    else
    {
      addSequential(new DriveLiftPIDZZZ(RobotMap.LIFT_LVL_1_POS,  RobotMap.LIFT_PID_SPEED_UP_MAX,
      RobotMap.LIFT_PID_SPEED_DOWN_MAX, RobotMap.LIFT_PID_SPEED_UP_MIN, RobotMap.LIFT_PID_SPEED_DOWN_MIN, 
      RobotMap.LIFT_PID_ERROR, true));
    }
}
}
