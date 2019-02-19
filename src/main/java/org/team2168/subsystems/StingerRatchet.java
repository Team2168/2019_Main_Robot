/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;
import org.team2168.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


public class StingerRatchet extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static DoubleSolenoid _ratchet;
  private static StingerRatchet _instance = null;

  public StingerRatchet()
  {
    _ratchet = new DoubleSolenoid(RobotMap.PCM_CAN_ID_BELLYPAN, 
      RobotMap.STINGER_RACHET_ENGAGE_PCM, RobotMap.STINGER_RACHET_DISENGAGE_PCM);
      ConsolePrinter.putBoolean("Stinger Ratchet Enagaged", () -> {return Robot.stingerRatchet.isRatchetEngaged();}, true, false);
      ConsolePrinter.putBoolean("Stinger Ratchet Disengaged", () -> {return Robot.stingerRatchet.isRatchetDisengaged();}, true, false);
  }

  /**
   * Calls instance object and makes it a singleton object of type DrivetrainStingerShifter
   * @return DrivetrainStingerShifter object "instance"
  */
  public static StingerRatchet getInstance()
  {
    if (_instance == null)
    {
      _instance = new StingerRatchet();
    }
    return _instance;
  }

  /**
   * Engages the stinger ratchet to lock stingers
   */
  public void engageRatchet()
  {
    _ratchet.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Disengages the stinger ratchet to allow the stingers to retract or extend
   */
  public void disengageRatchet()
  {
    _ratchet.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * 
   * @return true if last commanded shift was to engageRatchet
   */
  public boolean isRatchetEngaged()
  {
    return _ratchet.get() == DoubleSolenoid.Value.kForward;
  }
    
  /**
   * 
   * @return true if last commanded shift was to disengageRatchet
   */
  public boolean isRatchetDisengaged()
  {
    return _ratchet.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
