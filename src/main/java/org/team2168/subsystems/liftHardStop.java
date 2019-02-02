/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class liftHardStop extends Subsystem {
  private boolean _solenoidPos;
  private liftHardStop(){
    
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
  
private void baseExtendSolenoid(){
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

}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
