/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class StingerRatchet extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static DoubleSolenoid _ratchet;
  private static StingerRatchet _instance = null;

  public StingerRatchet()
  {
    _ratchet = new DoubleSolenoid(RobotMap.PCM_CAN_ID, 
      RobotMap.STINGER_RACHET_ENGAGE_PCM, RobotMap.STINGER_RACHET_DISENGAGE_PCM);
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

  public void engageRatchet()
  {
    _ratchet.set(DoubleSolenoid.Value.kForward);
  }

  public void disengageRatchet()
  {
    _ratchet.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isRatchetEngaged()
  {
    return _ratchet.get() == DoubleSolenoid.Value.kForward;
  }
    
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
