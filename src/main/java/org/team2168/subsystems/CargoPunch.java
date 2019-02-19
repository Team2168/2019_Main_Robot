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
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class CargoPunch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static DoubleSolenoid _cargoPunchPiston;
  private static CargoPunch _instance;

  private CargoPunch()
  {
    _cargoPunchPiston = new DoubleSolenoid(RobotMap.PCM_CAN_ID_LIFT, RobotMap.CARGO_PUNCH_EXTEND_PCM, RobotMap.CARGO_PUNCH_RETRACT_PCM);
    ConsolePrinter.putBoolean("Cargo Punch Extended", () -> {return Robot.cargoPunch.isExtended();}, true, false);
    ConsolePrinter.putBoolean("Cargo Punch Retracted", () -> {return Robot.cargoPunch.isRetracted();}, true, false);
  }

  /**
   * Calls instance object and makes it a singleton object of type CargoPunch
   * @return CargoPunch object "instance"
   */
  public static CargoPunch getInstance()
  {
    if (_instance == null)
    {
      _instance = new CargoPunch();
    }
    return _instance;
  }

  /**
   * Extends the cargo punch to shoot a cargo out of the intake
   */
  public void extendPunch()
  {
    _cargoPunchPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPunch()
  {
    _cargoPunchPiston.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * @return true if last commanded value was extendPunch
   */
  public boolean isExtended()
  {
    return _cargoPunchPiston.get() == DoubleSolenoid.Value.kForward;
  }

  /**
   * @return true if last commanded value was retractPunch
   */
  public boolean isRetracted()
  {
    return _cargoPunchPiston.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
