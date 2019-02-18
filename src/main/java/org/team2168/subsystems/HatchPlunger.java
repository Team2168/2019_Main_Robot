/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.AnalogInput;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

/**
 * Add your docs here.
 */
public class HatchPlunger extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid _PistonArm;
  private DoubleSolenoid _PistonPlunger;
  private AnalogInput _HatchSensor;
  //Constructor
  public HatchPlunger(){
    _PistonArm = new DoubleSolenoid(RobotMap.PCM_HATCHARM_ID, RobotMap.PLUNGER_EXTEND_PCM ,RobotMap.PLUNGER_RETRACT_PCM);
    _PistonPlunger = new DoubleSolenoid(RobotMap.PCM_HATCHPLUNGER_ID, RobotMap.PLUNGER_ENGAGE_PCM, RobotMap.PLUNGER_DISENGAGE_PCM);
    _HatchSensor = new AnalogInput(RobotMap.HATCH__INTAKE_IR_THRESHOLD );
        ConsolePrinter.putNumber("HatchPlunger Raw IR", () -> {return getRawIRVoltage();}, true, false);
        ConsolePrinter.putBoolean("Hatch is Present", () -> {return isHatchPresent();}, true, false);
        ConsolePrinter.putBoolean("Arm is extended", () -> {return isArmExtended();}, true, false);
        ConsolePrinter.putBoolean("Hatch is Engaged", () -> {return isHatchEngaged();}, true, false);
    }
  public void ExtendPlunger() {
    _PistonArm.set(DoubleSolenoid.Value.kForward);
  }
  public void RetractPlunger() {
    _PistonArm.set(DoubleSolenoid.Value.kReverse);
  }
  public void HatchEngaged() {
    _PistonPlunger.set(DoubleSolenoid.Value.kForward);
  }
	public void HatchDisengaged() {
    _PistonPlunger.set(DoubleSolenoid.Value.kReverse);
  }
  public double getRawIRVoltage(){
    return _HatchSensor.getVoltage();
  }
  public boolean isHatchPresent() {
      return (getRawIRVoltage() >= RobotMap.HATCH__INTAKE_IR_THRESHOLD);
  }
  public boolean isArmExtended(){
    return _PistonArm.get() == Value.kForward;
  }
  public boolean isArmRetracted(){
    return _PistonArm.get() == Value.kReverse;
  }
  public boolean isHatchEngaged(){
    return _PistonPlunger.get() == Value.kForward;
  }
  public boolean isHatchDisengaged(){
    return _PistonPlunger.get() == Value.kReverse;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
