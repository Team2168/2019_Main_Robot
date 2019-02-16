/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.team2168.robot.Robot;
import org.team2168.robot.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class DrivetrainStingerShifter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static DrivetrainStingerShifter _instance = null;
  private static DoubleSolenoid _drivetrainShifter; 

  /**
   * default constructor
   */
  private DrivetrainStingerShifter()
  {
    _drivetrainShifter = new DoubleSolenoid(RobotMap.PCM_CAN_ID, RobotMap.DRIVETRAIN_ENGAGED_PCM, RobotMap.STINGERS_ENGAGED_PCM);

    ConsolePrinter.putBoolean("Drivetrain Enagaged", () -> {return Robot.drivetrainStingerShifter.isDrivetrainEngaged();}, true, false);
		ConsolePrinter.putBoolean("Stingers Engaged", () -> {return Robot.drivetrainStingerShifter.isStingerEngaged();}, true, false);
  }

  /**
   * Calls instance object and makes it a singleton object of type DrivetrainStingerShifter
   * @return DrivetrainStingerShifter object "instance"
   */
  public static DrivetrainStingerShifter getInstance()
  {
    if (_instance == null)
    {
      _instance = new DrivetrainStingerShifter();
    }
    return _instance;
  }

  /**
   * Shifts the Drivetrain into engaged and the Stingers into neutral
   */
  public void engageDrivetrain()
  {
    _drivetrainShifter.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Shifts the Stingers into engaged and the Drivetrain into neutral
   */
  public void engageStingers()
  {
    _drivetrainShifter.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * @return true if last commanded shift was engageDrivetrain
   */
  public boolean isDrivetrainEngaged()
  {
    return _drivetrainShifter.get() == DoubleSolenoid.Value.kForward;
  }

  /**
   * @return true if last commanded shift was engageStingers
   */
  public boolean isStingerEngaged()
  {
    return _drivetrainShifter.get() == DoubleSolenoid.Value.kReverse;
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
