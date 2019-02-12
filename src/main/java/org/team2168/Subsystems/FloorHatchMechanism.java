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

  private static FloorHatchMechanism instance = null;
  private SpeedController runMotor;
  private DoubleSolenoid dsolenoidrotate;
  private static DigitalInput hallEffectRaise;
  private static DigitalInput hallEffectLower;
  private static final boolean reverseValue = false;

  private FloorHatchMechanism() {
    runMotor = new VictorSP(RobotMap.Hatch_Intake_Belt_CAN);
    dsolenoidrotate = new DoubleSolenoid(RobotMap.Hatch_Intake_Lower_pcm, RobotMap.Hatch_Intake_Raise_pcm);
    hallEffectRaise = new DigitalInput(RobotMap.Mechanism_Raise_DIO);
    hallEffectLower = new DigitalInput(RobotMap.Mechanism_Lower_DIO);
  }


  public void raise() {
    dsolenoidrotate.set(Value.kReverse);
  }

  public void lower() {
    dsolenoidrotate.set(Value.kForward);
  }
  //Positive values will cause the motor to spin.
  public void intakeHatchPanel(double speed) {
    if (reverseValue) {
      runMotor.set(-speed);
    }
    else {
      runMotor.set(speed);
    }
  
  }

  public boolean isSolenoidLowered(){
    return dsolenoidrotate.get() == Value.kForward;
  }

  public boolean isSolenoidRaised(){
    return dsolenoidrotate.get() == Value.kReverse;
  }

  public boolean isMechanismUp() {
    return !hallEffectRaise.get();
  }

    public boolean inMechanismLowered(){
      return !hallEffectLower.get();
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
