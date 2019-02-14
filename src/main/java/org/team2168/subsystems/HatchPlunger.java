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
import org.team2168.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchPlunger extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid doublesolenoidArm;
  private DoubleSolenoid doublesolenoidFingers;
  private AnalogInput HatchSensor;
  //Constructor
  public HatchPlunger(){
    doublesolenoidArm = new DoubleSolenoid(RobotMap.PLUNGER_EXTEND_PCM ,RobotMap.PLUNGER_RETRACT_PCM);
    doublesolenoidFingers = new DoubleSolenoid(RobotMap.PLUNGER_ENGAGE_PCM, RobotMap.PLUNGER_DISENGAGE_PCM);
    HatchSensor = new AnalogInput(RobotMap.HATCH__INTAKE_IR_THRESHOLD );
  }
  public void ExtendPlunger() {
    doublesolenoidArm.set(DoubleSolenoid.Value.kForward);
  }
  public void RetractPlunger() {
    doublesolenoidArm.set(DoubleSolenoid.Value.kReverse);
  }
  public void HatchEngaged() {
    doublesolenoidFingers.set(DoubleSolenoid.Value.kForward);
  }
	public void HatchDisengaged() {
    doublesolenoidFingers.set(DoubleSolenoid.Value.kReverse);
  }
  public double getRawIRVoltage(){
    return HatchSensor.getVoltage();
  }
  public boolean isHatchPresent() {
      return (getRawIRVoltage() >= RobotMap.HATCH__INTAKE_IR_THRESHOLD);
  }
  public boolean isArmExtended(){
    return doublesolenoidArm.get() == Value.kForward;
  }
  public boolean isArmRetracted(){
    return doublesolenoidArm.get() == Value.kReverse;
  }
  public boolean isHatchEngaged(){
    return doublesolenoidFingers.get() == Value.kForward;
  }
  public boolean isHatchDisengaged(){
    return doublesolenoidFingers.get() == Value.kReverse;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
