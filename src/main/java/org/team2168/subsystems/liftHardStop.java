/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.commands.LiftConstant;
import org.team2168.commands.LiftJoystick;
import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */


public class LiftHardStop extends Subsystem {

    /**
   * a double solenoid with a rubber pad on the end to stop a gear in the lift gearbox to "lock" the lift at the specified position
   */
  public static DoubleSolenoid liftDSolenoid;



  
  public LiftHardStop(){
    liftDSolenoid=new DoubleSolenoid(RobotMap.LIFT_BRAKE_ENGAGE_PCM, RobotMap.LIFT_BRAKE_DISENGAGE_PCM);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean getSolenoidPosition(){
    boolean _solenoidPos;
    if(liftDSolenoid.get()==Value.kForward){
      _solenoidPos=true;
    }
    else{
      _solenoidPos=false;
    }
    return _solenoidPos;
  }
  
private static void baseExtendSolenoid(){
  if(liftDSolenoid.get()!=Value.kForward){
    liftDSolenoid.set(Value.kForward);
  }
}
private static void baseContractSolenoid(){
  if(liftDSolenoid.get()!=Value.kReverse){
    liftDSolenoid.set(Value.kReverse);
  }
}
public static void extendSolenoid(){
  if (Lift.liftMotor1.get()==0.0 && Lift.liftMotor2.get()==0.0){
    baseExtendSolenoid();
  }
}

public static void contractSolenoid(){
  if (Lift.liftMotor1.get()==0.0 && Lift.liftMotor2.get()==0.0){
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
