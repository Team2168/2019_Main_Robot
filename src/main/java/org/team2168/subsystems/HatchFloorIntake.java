/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.hatchFloorIntake.DriveHatchFloorIntakeWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class HatchFloorIntake extends Subsystem 
{
  private Victor _intakeMotor;
  private DoubleSolenoid _dSolenoidRotate;

  private static HatchFloorIntake instance = null;
  public static volatile double _intakeMotorVoltage;

  private HatchFloorIntake() 
  {
    _intakeMotor = new Victor(RobotMap.HATCH_FLOOR_INTAKE_PDP);
    _dSolenoidRotate = new DoubleSolenoid(RobotMap.PCM_CAN_ID_BELLYPAN, RobotMap.HATCH_INTAKE_LOWER_PCM, RobotMap.HATCH_INTAKE_RAISE_PCM);

    // ConsolePrinter.putBoolean("Is Solenoid Raised",() -> {return isSolenoidRaised();}, true, false);
    // ConsolePrinter.putBoolean("Is Solenoid Lowered",() -> {return isSolenoidLowered();},true, false);
    // ConsolePrinter.putBoolean("Is HatchIntake Up",() -> {return isHatchIntakeUp();},true, false);
    // ConsolePrinter.putBoolean("Is HatchIntake Lowered",() -> {return isHatchIntakeLowered();},true, false);
    ConsolePrinter.putNumber("Intake motor voltage",() -> {return _intakeMotorVoltage;},true, false);
    ConsolePrinter.putBoolean("Intake Motor", () -> {return !Robot.pdp.isIntakeMotorTrip();}, true, false);
  }

  public static HatchFloorIntake getInstance()
  {
    if (instance == null)
      instance = new HatchFloorIntake();
    return instance;
  }

  public void raise()
  {
    _dSolenoidRotate.set(Value.kReverse);
  }

  public void lower()
  {
    _dSolenoidRotate.set(Value.kForward);
  }
  // Commander Cody, the time has come.
  // Positive values will cause the motor to spin.
  public void driveMotors(double speed)
  {
    if (RobotMap.HATCH_INTAKE_MOTOR_REVERSE)
      speed = -speed;

    _intakeMotor.set(speed);

    _intakeMotorVoltage = Robot.pdp.getBatteryVoltage() * speed;
  }

  public boolean isSolenoidLowered()
  {
    return _dSolenoidRotate.get() == Value.kForward;
  }

  public boolean isSolenoidRaised()
  {
    return _dSolenoidRotate.get() == Value.kReverse;
  }


  @Override
  public void initDefaultCommand()
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveHatchFloorIntakeWithJoystick());
  }
}

