/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.commands.FloorHatchMechanism.DriveWithJoystick;
import org.team2168.robot.RobotMap;
import org.team2168.robot.Robot;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class FloorHatchMechanism extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static FloorHatchMechanism instance = null;
  private SpeedController _runMotor;
  private DoubleSolenoid _dSolenoidRotate;
  private static DigitalInput _hallEffectRaise;
  private static DigitalInput _hallEffectLower;
  private static final boolean _reverseValue = false;
  public static volatile double _intakeMotorVoltage;

  private FloorHatchMechanism() {
    _runMotor = new VictorSP(RobotMap.Hatch_Intake_Belt_CAN);
    _dSolenoidRotate = new DoubleSolenoid(RobotMap.Hatch_Intake_Lower_pcm, RobotMap.Hatch_Intake_Raise_pcm);
    _hallEffectRaise = new DigitalInput(RobotMap.Mechanism_Raise_DIO);
    _hallEffectLower = new DigitalInput(RobotMap.Mechanism_Lower_DIO);

    ConsolePrinter.putBoolean("Is Solenoid Raised",() -> {return isSolenoidRaised();}, true, false);
    ConsolePrinter.putBoolean("Is Solenoid Lowered",() -> {return isSolenoidLowered();},true, false);
    ConsolePrinter.putBoolean("Is Mechanism Up",() -> {return isMechanismUp();},true, false);
    ConsolePrinter.putBoolean("Is Mechanism Lowered",() -> {return isMechanismLowered();},true, false);
    ConsolePrinter.putNumber("Intake motor voltage",() -> {return _intakeMotorVoltage;},true, false);
  }


  public void raise() {
    _dSolenoidRotate.set(Value.kReverse);
  }

  public void lower() {
    _dSolenoidRotate.set(Value.kForward);
  }
  //Positive values will cause the motor to spin.
  public void intakeHatchPanel(double speed) {
    if (_reverseValue) {
      _runMotor.set(-speed);
    }
    else {
      _runMotor.set(speed);
    }
    _intakeMotorVoltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  public boolean isSolenoidLowered(){
    return _dSolenoidRotate.get() == Value.kForward;
  }

  public boolean isSolenoidRaised(){
    return _dSolenoidRotate.get() == Value.kReverse;
  }

  public boolean isMechanismUp() {
    return !_hallEffectRaise.get();
  }

    public boolean isMechanismLowered(){
      return !_hallEffectLower.get();
    }
    public static FloorHatchMechanism getInstance(){
        if (instance == null)
          instance = new FloorHatchMechanism();
        return instance;
    }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoystick());
  }
}
// Commander Cody, the time has come.
