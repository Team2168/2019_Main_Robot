/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.CanAnalogInput;
import org.team2168.PID.sensors.CanDigitalInput;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchProbePistons extends Subsystem {

  private DoubleSolenoid _probePlungerPiston;
  private DoubleSolenoid _probeHatchEngagePiston;
  private CanAnalogInput _HatchSensor;

  private CanDigitalInput _limitSwitch;

  private static HatchProbePistons instance = null;

  private HatchProbePistons()
  {
    _probePlungerPiston = new DoubleSolenoid(RobotMap.PCM_CAN_ID_LIFT, RobotMap.PROBE_EXTEND_PCM,RobotMap.PROBE_RETRACT_PCM);
    _probeHatchEngagePiston = new DoubleSolenoid(RobotMap.PCM_CAN_ID_LIFT, RobotMap.PROBE_ENGAGE_PCM,RobotMap.PROBE_DISENGAGE_PCM);
    _HatchSensor = new CanAnalogInput(Robot.hatchProbePivot._plungerArmPivotMotor, CanAnalogInput.kSCALE_3_3_VOLTS);
    _limitSwitch = new CanDigitalInput(Robot.cargoIntakeWheels._intakeMotor);

    // ConsolePrinter.putNumber("HatchPlunger Raw IR", () -> {return getRawIRVoltage();}, true, false);
    // ConsolePrinter.putBoolean("Hatch is Present", () -> {return isHatchPresent();}, true, false);
    // ConsolePrinter.putBoolean("Arm is extended", () -> {return isArmExtended();}, true, false);
    // ConsolePrinter.putBoolean("Hatch is Engaged", () -> {return isHatchEngaged();}, true, false);
    ConsolePrinter.putBoolean("Is hatch present limit", () -> {return isHatchPresentLimitSwitch();}, true, false);


  }

  	/**
	 * Singleton constructor of the lift
	 * 
	 */

  public static HatchProbePistons getInstance()
  {
    if (instance == null)
      instance = new HatchProbePistons();
    return instance;
  }

  public void ExtendPlunger()
  {
    _probePlungerPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void RetractPlunger()
  {
    _probePlungerPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void HatchEngaged()
  {
    _probeHatchEngagePiston.set(DoubleSolenoid.Value.kForward);
  }

  public void HatchDisengaged()
  {
    _probeHatchEngagePiston.set(DoubleSolenoid.Value.kReverse);
  }

  public double getRawIRVoltage()
  {
    return _HatchSensor.getVoltage();
  }

  public boolean isHatchPresent()
  {
    if (Robot.isPracticeRobot())
      return (getRawIRVoltage() >= RobotMap.HATCH_INTAKE_IR_THRESHOLD_MAX_PBOT);
    else
      return (getRawIRVoltage() >= RobotMap.HATCH_INTAKE_IR_THRESHOLD_MAX);
  }

  public boolean isHatchPresentLimitSwitch()
  {
    return _limitSwitch.getForwardLimit();
  }

  public boolean isArmExtended()
  {
    return _probePlungerPiston.get() == Value.kForward;
  }

  public boolean isArmRetracted()
  {
    return _probePlungerPiston.get() == Value.kReverse;
  }

  public boolean isHatchEngaged()
  {
    return _probeHatchEngagePiston.get() == Value.kForward;
  }

  public boolean isHatchDisengaged()
  {
    return _probeHatchEngagePiston.get() == Value.kReverse;
  }

  @Override
  public void initDefaultCommand()
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
