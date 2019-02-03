/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.Subsystems;

import org.team2168.Commands.FloorHatchMechanism.DriveWithJoystick;
import org.team2168.robot.RobotMap;

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

  private SpeedController intakeMotor;
  private DoubleSolenoid dsolenoidrotate;
  private static DigitalInput MechanismUp;
  private static DigitalInput MechanismDown;

  public FloorHatchMechanism() {
    intakeMotor = new VictorSP(RobotMap.Hatch_Intake_Belt_CAN);
    dsolenoidrotate = new DoubleSolenoid(RobotMap.Hatch_Intake_Lower_pcm, RobotMap.Hatch_Intake_Raise_pcm);
    MechanismUp = new DigitalInput(RobotMap.Mechanism_Raise_DIO);
    MechanismDown = new DigitalInput(RobotMap.Mechanism_Lower_DIO);
  
  }

  public void raiseMechanism() {
    dsolenoidrotate.set(Value.kReverse);
  }

  public void lowerMechanism() {
    dsolenoidrotate.set(Value.kForward);
  }

  public void intakeHatchPanel(double speed) {
    intakeMotor.set(speed);
  }

  public boolean isMechanismExtended(){
    return dsolenoidrotate.get() == Value.kForward;
  }

  public boolean isMechanismRetracted(){
    return dsolenoidrotate.get() == Value.kReverse;
  }

 // ConsolePrinter.putBoolean("Is the mechanism up" , () -> ) {
  //  return FloorHatchMechanism1.isMechanismUp();
 // }

 // ConsolePrinter.putBoolean("Is the mechanism down" , () -> ) {
//    return Robot.FloorHatchMechanism1.isMechanismDown();
//  }

  public boolean isMechanismUp() {
    return !MechanismUp.get();
  }

  public boolean isMechanismDown() {
    return !MechanismDown.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoystick());
  }
}
// Commander Cody, the time has come.
