/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.commands.liftConstant;
import org.team2168.commands.liftJoystick;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class liftHardStop extends Subsystem {
  private boolean _solenoidPos;
  public liftHardStop(){
    
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean getSolenoidPosition(){
    if(lift.liftDSolenoid.get()==Value.kForward){
      _solenoidPos=true;
    }
    else{
      _solenoidPos=false;
    }
    return _solenoidPos;
  }
  
private static void baseExtendSolenoid(){
  if(lift.liftDSolenoid.get()!=Value.kForward){
    lift.liftDSolenoid.set(Value.kForward);
  }
}
private static void baseContractSolenoid(){
  if(lift.liftDSolenoid.get()!=Value.kReverse){
    lift.liftDSolenoid.set(Value.kReverse);
  }
}
public static void extendSolenoid(){
  if (lift.liftMotor1.get()==0.0 && lift.liftMotor2.get()==0.0){
    baseExtendSolenoid();
  }
}

public static void contractSolenoid(){
  if (lift.liftMotor1.get()==0.0 && lift.liftMotor2.get()==0.0){
    baseContractSolenoid();


    }
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
    ;
  }
}
