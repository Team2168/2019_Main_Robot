/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShifterDrivetrain extends Subsystem {
  private static ShifterDrivetrain _instance = null;
  private static Solenoid _drivetrainShifter; 


  private ShifterDrivetrain()
  {
    _drivetrainShifter = new Solenoid(RobotMap.PCM_CAN_ID_BELLYPAN,  RobotMap.DRIVETRAIN_ENGAGED_PCM);

    ConsolePrinter.putBoolean("Drivetrain Enagaged", () -> {return Robot.shifterDrivetrain.isDrivetrainDisengaged();}, true, false);
    ConsolePrinter.putBoolean("Drivetrain Disnagaged", () -> {return Robot.shifterDrivetrain.isDrivetrainDisengaged();}, true, false);
	
  }

  /**
   * Calls instance object and makes it a singleton object of type DrivetrainStingerShifter
   * @return DrivetrainStingerShifter object "instance"
   */
  public static ShifterDrivetrain getInstance()
  {
    if (_instance == null)
    {
      _instance = new ShifterDrivetrain();
    }
    return _instance;
  }

  /**
   * Shifts the Drivetrain into engaged and the Stingers into neutral
   */
  public void engageDrivetrain()
  {
    Robot.isClimbEnabled = false;
    _drivetrainShifter.set(false);
  }

  /**
   * Shifts the Stingers into engaged and the Drivetrain into neutral
   */
  public void disengageDrivetrain()
  {
    _drivetrainShifter.set(true);
  }


  /**
   * @return true if last commanded shift was engageDrivetrain
   */
  public boolean isDrivetrainEngaged()
  {
    return _drivetrainShifter.get() == false;
  }

  public boolean isDrivetrainDisengaged()
  {
    return _drivetrainShifter.get() == true;
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
