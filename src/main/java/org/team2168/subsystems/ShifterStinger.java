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

public class ShifterStinger extends Subsystem {
  private static ShifterStinger _instance = null; 
  private static DoubleSolenoid _stingerShifter;

  private ShifterStinger()
  {
    _stingerShifter = new DoubleSolenoid(RobotMap.PCM_CAN_ID_BELLYPAN, RobotMap.STINGER_ENGAGE_PCM, RobotMap.STINGER_DISENGAGE_PCM);


   	// ConsolePrinter.putBoolean("Stingers Engaged", () -> {return Robot.shifterStinger.isStingerEngaged();}, true, false);
    // ConsolePrinter.putBoolean("Stingers Disengaged", () -> {return Robot.shifterStinger.isStingerDisengaged();}, true, false);

  }

  /**
   * Calls instance object and makes it a singleton object of type DrivetrainStingerShifter
   * @return DrivetrainStingerShifter object "instance"
   */
  public static ShifterStinger getInstance()
  {
    if (_instance == null)
    {
      _instance = new ShifterStinger();
    }
    return _instance;
  }


  public void engageStingers()
  {
    Robot.isClimbEnabled = true;
    _stingerShifter.set(Value.kForward);
  }

  public void disengageStingers()
  {
    Robot.isClimbEnabled = false;
    Robot.isClimbEnabledLevel2 = false;
    _stingerShifter.set(Value.kReverse);
  }


  /**
   * @return true if last commanded shift was engageStingers
   */
  public boolean isStingerEngaged()
  {
    
    return _stingerShifter.get() == Value.kForward;
  }

  public boolean isStingerDisengaged()
  {
    return _stingerShifter.get() == Value.kReverse;
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
